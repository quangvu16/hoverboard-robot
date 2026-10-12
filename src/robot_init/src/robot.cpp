#include "robot_init/robot.hpp"
#include <nlohmann/json.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <thread>
#include <vector>

#include <tf2/utils.h>
#include <geometry_msgs/msg/twist.hpp>
#include <angles/angles.h>

using std::placeholders::_1;

ROBOT::ROBOT(const nlohmann::json &robot_data)
    : Node("robot_module"),
      status_(this), task_(this), subtask(this), slam_(this), error_(this), bus_(this), nav_(this, &bus_),
      is_onsite(true), is_moving(false), is_task(false), is_slam(false), is_login(false), 
      is_error(false), is_cancel(false), is_manual(false)
{
    // t?o ID t?m
    auto now = std::chrono::system_clock::now();
    auto in_time = std::chrono::system_clock::to_time_t(now);
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
    std::stringstream ss;
    ss << "PackageRobotID_" << std::put_time(std::localtime(&in_time), "%Y%m%d_%H.%M.%S_")
       << std::setw(6) << std::setfill('0') << micros;
    id_robot_ = ss.str();

    // c?u hình
    serial_      = robot_data["serial"];
    device_      = robot_data["Device"];
    ip_server_   = robot_data["server_ip"];
    server_port_ = robot_data["server_port"];
    ip_robot     = robot_data["robot_ip"];
    robot_port   = robot_data["robot_port"];
    float x = robot_data.value("location_x", 0.0f);
    float y = robot_data.value("location_y", 0.0f);
    location = {x, y};

    // network + bus + nav
    net_ = std::make_unique<NetClient>(ip_server_, server_port_);

    // TF n?u còn c?n
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // gi? l?i publisher g?i ý pose
    suggest_pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/suggest_pose", 10);

    // sub thông báo thành công t? controller ngoài, gi? nguyên logic
    sub_success_ = this->create_subscription<std_msgs::msg::Bool>(
      "/nav_success", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr m){
        if (nav_active_ && m->data) {
          RCLCPP_INFO(get_logger(), "straight_controller báo SUCCESS");
          nav_active_ = false;
          is_moving = false;
          bus_.move_type("none");
        }
      });

    register_with_server();

}

// ===== codec wrappers =====

std::optional<nlohmann::json> ROBOT::extract_data(std::vector<uint8_t>& buffer){ return codec_.extract_data(buffer); }

// ===== net/register =====
void ROBOT::register_with_server() {
  using namespace std::chrono_literals;

  // khung RegDevice d?ng s?n
  nlohmann::json reg = {
    {"Report", "RegDevice"},
    {"Device", device_},
    {"ID", id_robot_},
    {"Time", static_cast<int>(time(nullptr))},
    {"Body", {
      {"Device", {
        {"Ip",   ip_robot},
        {"Port", robot_port},
        {"SerialNo", device_}
      }}
    }}
  };
  auto reg_frame = codec_.build_tcp_message(reg);

  // nh?n d? li?u ? feed codec ? handle
  net_->start([this](const NetClient::Bytes& chunk){
    rx_buffer_.insert(rx_buffer_.end(), chunk.begin(), chunk.end());
    for (;;) {
      auto j = codec_.extract_data(rx_buffer_);
      if (!j) break;
      try { handle_message(*j); } catch (...) {}
    }
  });

  // g?i RegDevice m?i l?n connect/reconnect
  net_->set_on_connect([this, reg_frame](){
    (void)net_->send_all(reg_frame);
  });

  // reset khi disconnect
  net_->set_on_disconnect([this](){
    reset_state_on_disconnect();
  });

  // th? g?i ngay n?u dã connected
  if (net_->connected()) (void)net_->send_all(reg_frame);
}

void ROBOT::send_to_server(const nlohmann::json& msg){
  auto frame = codec_.build_tcp_message(msg);
  if (net_->send_all(frame)) return;
  for (int i=0;i<20 && !net_->connected();++i)
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  net_->send_all(frame);
}

bool ROBOT::send_all(const uint8_t* data, size_t len) {
    return net_->send_all(std::vector<uint8_t>(data, data + len));
}

void ROBOT::listen_server() { /* no-op, dùng NetClient */ }

// ===== handlers =====
void ROBOT::handle_message(const nlohmann::json &msg)
{
  std::string device_name = msg.at("Device");
  //if (device_name == device_)
  //  {
      std::string action = msg.value("Action", msg.value("Report", ""));
      if (msg.contains("id"))
          std::cout << "[DEBUG] Server ID: " << msg["id"] << std::endl;

      if (action == "loginDevice") {
          func_login(msg);
      } else if (action == "HeartBeating") {
          std::cout << "Hearbeat from Server" << std::endl;
          std::thread(&ROBOT::func_heartbeat, this, msg).detach();
      } else if (action == "addTask") {
            func_add_task_response(msg);
            if (!is_error && !is_manual) {
              is_cancel = false;
              status_.state = "InTask";
              status_.task_state = "RUNNING";
              std::thread(&STATUS::update_task_status, &status_).detach();
          }

      } else if (action == "cancelTask") {

          func_cancel_task_response(msg);
         if (status_.state == "InTask"){
            status_.state = "Idle";
            status_.task_state = "Canceled";
            bus_.cancel_task(0);
            is_cancel = true;
          }

      } else if (action == "addTaskSlice") {
          func_add_task_slice_response(msg);
          std::thread(&ROBOT::func_handle_task_slice, this, msg).detach();
      } else if (action == "getPointCloudData") {
          func_correct_location_response(msg);
          std::thread(&ROBOT::func_correct_location, this, msg).detach();
      } else if (action == "FreeMove") {
          std::thread(&ROBOT::func_add_task_response, this, msg).detach();
          std::thread(&ROBOT::func_free_move, this, msg).detach();
      } else if (action == "correctLocation") {
          // no-op
      } else if (action == "scanMap") {
          func_scanMap_task_response(msg);
          slam_.start();
          is_slam = true;
      } else if (action == "manualMove") {
          std::thread(&ROBOT::func_manualMove_response, this, msg).detach();
          std::thread(&ROBOT::func_manualMove, this, msg).detach();
      } else {
          std::cerr << "Unknown Action: " << action << std::endl;
      }
  //  }
}

void ROBOT::func_login(const nlohmann::json &server_msg)
{
    if (!server_msg.contains("ID")) {
        std::cerr << "Server message missing ID field\n";
        return;
    }

    std::string server_id = server_msg["ID"];
    nlohmann::json reply = {
        {"Report", "loginDevice"},
        {"code", 1000},
        {"data", {
            {"params", {
                {"Device", {
                    {"Ip", ip_robot},
                    {"Port", robot_port},
                    {"SerialNo", device_}
                }}
            }},
            {"result", true}
        }},
        {"desc", "SUCCESS"},
        {"device", device_},
        {"ID", server_id},
        {"time", static_cast<int>(time(nullptr))}
    };

    auto framed = codec_.build_tcp_message(reply);
    net_->send_all(framed);
    is_login = true;
    std::thread(&STATUS::update_device_status, &status_).detach();
    
}

void ROBOT::func_heartbeat(const nlohmann::json &server_msg)
{
    nlohmann::json reply = {
        {"Report", "HeartBeating"},
        {"device", device_},
        {"id", server_msg["ID"]},
        {"time", static_cast<int>(time(nullptr))},
        {"data", nullptr}
    };
    auto framed = codec_.build_tcp_message(reply);
    net_->send_all(framed);
}

void ROBOT::func_add_task_response(const nlohmann::json &msg)
{
    std::string desc;
    if (is_error || is_manual)
        desc =  "failed" ;
    else
        desc =  "SUCCESS" ;

    std::string server_id = msg["ID"];
    nlohmann::json reply = {
        {"Report", "addTask"},
        {"code", 1000},
        {"desc", desc},
        {"device", device_},
        {"id", server_id},
        {"time", static_cast<int>(time(nullptr))},
        {"data", {{"FailedList", {{{"TaskID",""},{"Error",""}}}}, {"result", true}}}
    };
    auto framed = codec_.build_tcp_message(reply);
    net_->send_all(framed);
}

void ROBOT::func_cancel_task_response(const nlohmann::json &msg)
{
    std::string server_id = msg["ID"];
    nlohmann::json reply = {
        {"Report", "cancelTask"},
        {"code", 1000},
        {"desc", "SUCCESS"},
        {"device", device_},
        {"id", server_id},
        {"time", static_cast<int>(time(nullptr))},
        {"data", {{"result", true}}}
    };
    auto framed = codec_.build_tcp_message(reply);
    net_->send_all(framed);
    
    reply = {
                {"Report", "Event"},
                {"Device", device_},
                {"ID", id_robot_},
                {"Time", static_cast<int>(time(nullptr))},
                {"Body", {                                       
                  {"EventCode", "RobotRequestCancelTask"},
                  {"EventAction", "Pulse"},
                  {"EventIndex", "meaningless"},
                  {"EventData", {
                       {"ID", id_robot_},
                       {"TaskID", task_.taskID}                                                    
                }}
                }}
            };
    
    framed = codec_.build_tcp_message(reply);
    net_->send_all(framed);
    
    
}

void ROBOT::func_scanMap_task_response(const nlohmann::json &msg)
{
    std::string server_id = msg["ID"];
    nlohmann::json reply = {
        {"Report", "scanMap"},
        {"code", 1000},
        {"desc", "SUCCESS"},
        {"device", device_},
        {"id", server_id},
        {"time", static_cast<int>(time(nullptr))},
        {"data", {{"result", true}}}
    };
    auto framed = codec_.build_tcp_message(reply);
    net_->send_all(framed);
}

void ROBOT::func_add_task_slice_response(const nlohmann::json &msg)
{
    std::string server_id = msg["ID"];
    nlohmann::json reply = {
        {"Report", "addTaskSlice"},
        {"code", 1000},
        {"desc", "SUCCESS"},
        {"device", device_},
        {"id", server_id},
        {"time", static_cast<int>(time(nullptr))},
        {"data", {{"FailedList", {{{"TaskID",""},{"Error",""}}}}, {"result", true}}}
    };
    auto framed = codec_.build_tcp_message(reply);
    net_->send_all(framed);
}

void ROBOT::func_correct_location_response(const nlohmann::json &msg)
{
    std::string server_id = msg["ID"];
    nlohmann::json reply = {
        {"Report", "getPointCloudData"},
        {"code", 1000},
        {"desc", "SUCCESS"},
        {"device", device_},
        {"id", server_id},
        {"time", static_cast<int>(time(nullptr))},
        {"data", {{"FailedList", {{{"TaskID",""},{"Error",""}}}}, {"result", true}}}
    };
    auto framed = codec_.build_tcp_message(reply);
    net_->send_all(framed);
}

void ROBOT::func_handle_task_slice(const nlohmann::json &msg) {
    subtask.subtask_handle(msg);
}

std::string ROBOT::get_id_by_position(const std::vector<float> loc) {
    std::string id = RobotFunction::get_id_by_position(loc);
    if(id!="") node_id = id;
    return id;
}

void ROBOT::get_position_by_id(const std::string id) {
    auto pos_opt = RobotFunction::get_position_by_id(id);
    if (!pos_opt.has_value()) {
        std::cerr << "[FreeMove] Node ID not found in topology: " << id << "\n";
        return;
    }
    location[0] = pos_opt->first;
    location[1] = pos_opt->second;
}

void ROBOT::func_correct_location(const nlohmann::json &msg)
{
    if (!msg.contains("Body")) { std::cerr << "[correctLocation] Missing Body\n"; return; }
    try {
        const auto &body = msg["Body"];
        double x_mm = body.at("Position").at(0).get<double>();
        double y_mm = body.at("Position").at(1).get<double>();
        float x = static_cast<float>(x_mm / 1000.0);
        float y = static_cast<float>(y_mm / 1000.0);

        std_msgs::msg::Float64MultiArray pose_msg;
        pose_msg.data.push_back(static_cast<double>(x));
        pose_msg.data.push_back(static_cast<double>(y));
        suggest_pose_pub_->publish(pose_msg);

        std::cout << "[correctLocation] Published corrected pose: x=" << x
                  << ", y=" << y << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "[correctLocation] Exception: " << e.what() << "\n";
    }
}

void ROBOT::func_manualMove_response(const nlohmann::json &msg)
{
    std::string server_id = msg["ID"];
    nlohmann::json reply = {
        {"Report", "manualMove"},
        {"code", 1000},
        {"desc", "SUCCESS"},
        {"device", device_},
        {"id", server_id},
        {"time", static_cast<int>(time(nullptr))},
        {"data", {{"result", true}}}
    };
    auto framed = codec_.build_tcp_message(reply);
    net_->send_all(framed);
}

void ROBOT::func_manualMove(const nlohmann::json &msg)
{
    if (!msg.contains("Body")) {
        std::cerr << "[manualMove] Missing Body\n";
        return;
    }
    try {
        const auto& vel = msg["Body"]["Task"][0];
        double lin = vel["LinearVelocity"].get<int>()/1000.0f;
        double ang = vel["AngularVelocity"].get<int>()/1000.0f;
        geometry_msgs::msg::Twist twist;
        twist.angular.z = ang;
        twist.linear.x = lin;
        // n?u dã chuy?n /cmd_vel vào NavAdapter, thêm API stop/send; ? dây t?m publish tr?c ti?p n?u còn gi? pub
        // n?u không gi?, chuy?n sang nav_->publish_cmd_vel(twist);
        if (cmd_vel_pub_) cmd_vel_pub_->publish(twist);
    } catch (const std::exception &e) {
        std::cerr << "[manualMove] Exception: " << e.what() << "\n";
    }
}

void ROBOT::func_free_move(const nlohmann::json &msg)
{
    try {
        if (!msg.contains("Body")) { std::cerr << "[FreeMove] Missing 'Body'\n"; return; }
        const auto& body = msg["Body"];
        if (!body.contains("Task") || !body["Task"].is_array() || body["Task"].empty()) {
            std::cerr << "[FreeMove] Missing or invalid 'Task'\n"; return;
        }
        const auto& t = body["Task"][0];
        if (!t.contains("EndPoint")) { std::cerr << "[FreeMove] Task missing 'EndPoint'\n"; return; }

        std::string node_id;
        if (t["EndPoint"].is_string()) node_id = t["EndPoint"].get<std::string>();
        else if (t["EndPoint"].is_number_integer()) node_id = std::to_string(t["EndPoint"].get<int>());
        else { std::cerr << "[FreeMove] EndPoint type invalid\n"; return; }

        auto pos_opt = RobotFunction::get_position_by_id(node_id);
        if (!pos_opt.has_value()) { std::cerr << "[FreeMove] Node not found: " << node_id << "\n"; return; }

        float x = pos_opt->first / 1000.0f;
        float y = pos_opt->second / 1000.0f;

        nav_.publish_goal(x, y, 0.0);
        std::cout << "[FreeMove] Published NAV goal to " << node_id
                  << ": x=" << x << ", y=" << y << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "[FreeMove] Exception: " << e.what() << std::endl;
    }
}

void ROBOT::func_free_move_response(const nlohmann::json &msg)
{
    try {
        if (!msg.contains("Body")) { std::cerr << "[FreeMove] Missing 'Body'\n"; return; }
        const auto& body = msg["Body"];
        if (!body.contains("Task") || !body["Task"].is_array() || body["Task"].empty()) {
            std::cerr << "[FreeMove] Missing or invalid 'Task'\n"; return;
        }
        const auto& t = body["Task"][0];
        if (!t.contains("EndPoint")) { std::cerr << "[FreeMove] Task missing 'EndPoint'\n"; return; }

        std::string node_id;
        if (t["EndPoint"].is_string()) node_id = t["EndPoint"].get<std::string>();
        else if (t["EndPoint"].is_number_integer()) node_id = std::to_string(t["EndPoint"].get<int>());
        else { std::cerr << "[FreeMove] EndPoint type invalid\n"; return; }

        auto pos_opt = RobotFunction::get_position_by_id(node_id);
        if (!pos_opt.has_value()) { std::cerr << "[FreeMove] Node not found: " << node_id << "\n"; return; }

        float x = pos_opt->first / 1000.0f;
        float y = pos_opt->second / 1000.0f;
        nav_.publish_goal(x, y, 0.0);
    }
    catch (const std::exception& e) {
        std::cerr << "[FreeMove] Exception: " << e.what() << std::endl;
    }
}

// ===== high-level publish via Bus/NavAdapter =====
void ROBOT::publish_charge_cmd(bool charge)
{
    bus_.charge(charge);
    nlohmann::json task_status = {
        {"desc", "TASK SUCCESS"},
        {"id", task_.taskID},
        {"time", static_cast<int>(time(nullptr))}
    };
    status_.task_state = "Finished";
    status_.state = "InCharging";
}

void ROBOT::navigateThroughWaypoints(const std::vector<nlohmann::json>& waypoints_json) {
  bus_.move_type("moveUp");
  nav_active_ = true;
  nav_.follow_waypoints_json(waypoints_json, task_.release_scan_mode);
}

void ROBOT::navigateThroughWaypointsForward(const std::vector<nlohmann::json>& waypoints_json) {
  bus_.move_type("moveUp");
  nav_active_  = true;
  nav_.follow_waypoints_json(waypoints_json, task_.release_scan_mode);
}

void ROBOT::navigateThroughWaypointsBackward(const std::vector<nlohmann::json>& waypoints_json) {
  bus_.move_type("moveDown");
  nav_active_  = true;
  nav_.follow_waypoints_json(waypoints_json, task_.release_scan_mode);
}

void ROBOT::publish_action(const std::string &action){
    bus_.action(action);
}

void ROBOT::publish_release_scan(bool trigger){
    bus_.release_scan(trigger);
}

void ROBOT::publish_cancel_task(int type) {
    bus_.cancel_task(static_cast<int16_t>(type));
}

// ===== reset state on disconnect =====
void ROBOT::reset_state_on_disconnect() {
  rx_buffer_.clear();
  nav_active_ = false;
  is_moving   = false;
  is_task     = false;
  is_slam     = false;
  is_login    = false;
  task_.release_scan = false;
  task_.release_scan_mode = false;

  bus_.move_type("none");
  bus_.action("none");
  status_.task_state = "Idle";
  //status_.state = "Disconnected";
  bus_.state(status_.state);
}

void ROBOT::robot_cancel_task()
{
  status_.task_state = "Cancel";
}
