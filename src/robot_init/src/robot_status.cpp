#include "robot_init/robot_status.hpp"
#include "robot_init/robot.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <sys/socket.h>  // 🔧 Required for send()

using json = nlohmann::json;

STATUS::STATUS(ROBOT* robot) : robot_(robot), tf_buffer_(robot_->get_clock()),
    tf_listener_(tf_buffer_) {
    initialpose_sub_ = robot_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        std::bind(&STATUS::initialpose_callback, this, std::placeholders::_1)
    );
//    amcl_pose_sub_ = robot_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//        "/amcl_pose", 10,
//        std::bind(&STATUS::amcl_pose_callback, this, std::placeholders::_1)
//    );

    battery_sub_ = robot_->create_subscription<std_msgs::msg::Int32>(
        "/battery_capacity", 10,
        std::bind(&STATUS::battery_callback, this, std::placeholders::_1)
    );
    
    task_status_sub_ = robot_->create_subscription<std_msgs::msg::String>(
      "/task_status", 10, std::bind(&STATUS::taskStatusCallback, this, std::placeholders::_1));
      
    task_release_distance_sub_ = robot_->create_subscription<std_msgs::msg::Bool>(
      "/task/release_nearest", 10, std::bind(&STATUS::releaseNearestCallback, this, std::placeholders::_1));
      
    manual_sub_ = robot_->create_subscription<std_msgs::msg::Bool>(
      "/manual_state", 10, std::bind(&STATUS::manualCallback, this, std::placeholders::_1));
      
    confirm_sub_ = robot_->create_subscription<std_msgs::msg::Bool>(
      "/confirm_state", 10, std::bind(&STATUS::confirmCallback, this, std::placeholders::_1));

    bumper_sub_ = robot_->create_subscription<std_msgs::msg::Bool>(
      "/bumper/status", 10, std::bind(&STATUS::bumperCallback, this, std::placeholders::_1));
          
    tf_timer_ = robot_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&STATUS::update_from_tf, this));
}

void STATUS::update_device_status() {
    while (robot_->is_login) {
        json msg = {
            {"Report", "DeviceState"},
            {"Body", {
                {"State", {
                    {"AngularVelocity", 0},
                    {"Battery", robot_->battery},
                    {"BatteryOutput", {
                        {"Current", 1},
                        {"Voltage", 0}
                    }},
                    {"ExtraStateFirst", 0},
                    {"ExtraStateSecond", 0},
                    {"Last2DCode", robot_->node_id},
                    {"MapId", robot_->map_name},
                    {"MotorVoltage", 0},
                    {"Odom", 0},
                    {"Oritation", static_cast<int>(robot_->orientation * 1000)},
                    {"PayLoad", 0},
                    {"Position", {robot_->location[0], robot_->location[1]}},
                    {"Speed", 0},
                    {"State", state},
                }}
            }},
            {"Device", robot_->device_},
            {"ID", robot_->id_robot_},
            {"Time", static_cast<int>(time(nullptr))}
        };
        if (!robot_->is_slam) {
          auto framed = robot_->codec_.build_tcp_message(msg);
          robot_->net_->send_all(framed);
        }
        robot_->bus_.state(state);
        robot_->bus_.nodeID(current_nodeID);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void STATUS::update_task_status() {
    while (robot_->is_login) {
    current_task_state = task_state;
    //std::cout << "current_task_State:" << current_task_state << std::endl;
      if(in_task_state) {
            json msg = {
                {"Report", "TaskState"},
                {"Device", robot_->device_},
                {"ID", robot_->id_robot_},
                {"Time", static_cast<int>(time(nullptr))},
                {"Body", {
                    {"States", {
                        {
                            {"ErrorNumber", 0},
                            {"Reason", ""},
                            {"ReasonDesc", ""},
                            {"RemainTime", 0},
                            {"State", current_task_state},
                            {"TaskID", robot_->task_.taskID}
                        }
                    }}
                }}
            };
    
            auto framed = robot_->codec_.build_tcp_message(msg);
            robot_->net_->send_all(framed);
     }
        if (current_task_state == "RUNNING")
            in_task_state = true;
        else if (current_task_state == "Finished" || current_task_state == "Cancel" )
            in_task_state = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void STATUS::initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    const auto& pose = msg->pose.pose;
    std::vector<float> loc = {0.0, 0.0};  // Or init from json

    loc[0] = static_cast<int>(1000*static_cast<float>(pose.position.x));
    loc[1] = static_cast<int>(1000*static_cast<float>(pose.position.y));

    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );
    double roll, pitch, yaw_rad;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_rad);

    double yaw_deg = fmod((yaw_rad * 180.0 / M_PI) + 360.0, 360.0);
    robot_->orientation = static_cast<float>(yaw_deg);
    std::string id = robot_->get_id_by_position(loc);
    
    if (id!="")
    {
        // 3. Lookup node position
        std::cout << "node id: "<< id << std::endl;
        robot_->node_id = id;
        current_nodeID = id;
        robot_->get_position_by_id(id);

    }

    else
       {
        current_nodeID = "None";
        robot_->location[0] = loc[0];
        robot_->location[1] = loc[1];
    }

 /*   std::cout << "[STATUS] Updated from /initialpose: x=" << robot_->location[0]
              << ", y=" << robot_->location[1]
              << ", yaw=" << yaw_deg << " rad\n";*/
}

/*void STATUS::amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    const auto& pose = msg->pose.pose;
    std::vector<float> loc = {0.0, 0.0};  // Or init from json

    loc[0] = static_cast<int>(1000*static_cast<float>(pose.position.x));
    loc[1] = static_cast<int>(1000*static_cast<float>(pose.position.y));

    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );
    double roll, pitch, yaw_rad;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_rad);

    double yaw_deg = fmod((yaw_rad * 180.0 / M_PI) + 360.0, 360.0);
    robot_->orientation = static_cast<float>(yaw_deg);
    std::string id = robot_->get_id_by_position(loc);
    
    if (id!="")
    {
        // 3. Lookup node position
        std::cout << "node id: "<< id << std::endl;
        robot_->node_id = id;
        robot_->get_position_by_id(id);

    }

    else
       {
        robot_->location[0] = loc[0];
        robot_->location[1] = loc[1];
    }


    // std::cout << "[STATUS] Updated from /amcl_pose -> x: " << robot_->location[0]
    //           << ", y: " << robot_->location[1]
    //           << ", yaw: " << yaw_deg << " degrees" << std::endl;
}*/

void STATUS::battery_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    robot_->battery = msg->data;
 // if (msg->state == "discharging")
 //     state = "Idle";
 // else
 //     state = msg->state;
}

void STATUS::taskStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data != "success") return;
  else 
  {
      robot_->is_task = false;
      robot_->subtask.completed_subtask(); 
      if (robot_->task_.release_scan_mode)
          robot_->task_.release_scan_mode = false;
  }
}

void STATUS::releaseNearestCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if(msg->data)
      robot_->task_.release_scan = !msg->data;
}

void STATUS::manualCallback(const std_msgs::msg::Bool::SharedPtr msgs) {
  robot_->is_manual = msgs->data;
    std::string action;
    if (msgs->data) 
        action = "Start";
    else
        action = "Stop";
if (action != manual_action && action == "Start"){
    manual_action = action;
    json msg = {
                {"Report", "Event"},
                {"Device", robot_->device_},
                {"ID", robot_->id_robot_},
                {"Time", static_cast<int>(time(nullptr))},
                {"Body", {                                       
                  {"EventCode", "RobotManualIntervention"},
                  {"EventAction", "Start"},
                  {"EventIndex", "meaningless"},
                  {"EventHandler", "NULL"},
                  {"EventData", {
                       {"id", robot_->id_robot_},
                       {"Position", {robot_->location[0], robot_->location[1]}}                                                    
                }}
                }}
            };
    
    auto framed = robot_->codec_.build_tcp_message(msg);
    robot_->net_->send_all(framed);
    ManualIntervention = true;
 }
    manual_action = action;
}

void STATUS::confirmCallback(const std_msgs::msg::Bool::SharedPtr msgs) {

if ((msgs->data && manual_action == "Stop" && ManualIntervention)
      ||(msgs->data && robot_->is_error && ManualIntervention)){
    json msg = {
                {"Report", "Event"},
                {"Device", robot_->device_},
                {"ID", robot_->id_robot_},
                {"Time", static_cast<int>(time(nullptr))},
                {"Body", {                                       
                  {"EventCode", "RobotManualIntervention"},
                  {"EventAction", "Stop"},
                  {"EventIndex", "meaningless"},
                  {"EventHandler", "NULL"},
                  {"EventData", {
                       {"id", robot_->id_robot_},
                       {"Position", {robot_->location[0], robot_->location[1]}}                                                    
                }}
                }}
            };
    
    auto framed = robot_->codec_.build_tcp_message(msg);
    robot_->net_->send_all(framed);
    ManualIntervention = false;
    robot_->is_error = false;
    robot_->bus_.error(false);
 }

}


void STATUS::bumperCallback(const std_msgs::msg::Bool::SharedPtr msgs) {

if(msgs->data != last_bumper){
    last_bumper = msgs->data;

    if(msgs->data) {
        json msg = {
                {"Report", "Event"},
                {"Device", robot_->device_},
                {"ID", robot_->id_robot_},
                {"Time", static_cast<int>(time(nullptr))},
                {"Body", {                                       
                  {"EventCode", "RobotManualIntervention"},
                  {"EventAction", "Start"},
                  {"EventIndex", "meaningless"},
                  {"EventHandler", "NULL"},
                  {"EventData", {
                       {"id", robot_->id_robot_},
                       {"Position", {robot_->location[0], robot_->location[1]}}                                                    
                }}
                }}
            };
    
      auto framed = robot_->codec_.build_tcp_message(msg);
      robot_->net_->send_all(framed);
    
      msg = {
                  {"Report", "Event"},
                  {"Device", robot_->device_},
                  {"ID", robot_->id_robot_},
                  {"Time", static_cast<int>(time(nullptr))},
                  {"Body", {                                       
                    {"EventCode", "RobotCollision"},
                    {"EventAction", "Start"},
                    {"EventIndex", "meaningless"},
                    {"EventHandler", "NULL"},
                    {"EventData", {
                         {"id", robot_->id_robot_},
                         {"Position", {robot_->location[0], robot_->location[1]}}                                                    
                  }}
                  }}
              };
      
      framed = robot_->codec_.build_tcp_message(msg);
      robot_->net_->send_all(framed);
      ManualIntervention = true;

    }
  else {
        json msg = {
                    {"Report", "Event"},
                    {"Device", robot_->device_},
                    {"ID", robot_->id_robot_},
                    {"Time", static_cast<int>(time(nullptr))},
                    {"Body", {                                       
                      {"EventCode", "RobotCollision"},
                      {"EventAction", "Stop"},
                      {"EventIndex", "meaningless"},
                      {"EventHandler", "NULL"},
                      {"EventData", {
                           {"id", robot_->id_robot_},
                           {"Position", {robot_->location[0], robot_->location[1]}}                                                    
                    }}
                    }}
                };
        
        auto framed = robot_->codec_.build_tcp_message(msg);
        robot_->net_->send_all(framed);
      }
  }
}

void STATUS::update_from_tf(){
    try {
      auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

      const auto &t = tf.transform.translation;
      const auto &q = tf.transform.rotation;

      // mm like your AMCL code
      std::vector<float> loc = {0.0f, 0.0f};
      loc[0] = static_cast<int>(1000.0f * static_cast<float>(t.x));
      loc[1] = static_cast<int>(1000.0f * static_cast<float>(t.y));

      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      double roll, pitch, yaw_rad;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw_rad);
      double yaw_deg = std::fmod((yaw_rad * 180.0 / M_PI) + 360.0, 360.0);
      robot_->orientation = static_cast<float>(yaw_deg);

      std::string id = robot_->get_id_by_position(loc);
      
      if (id!="")
        {
          robot_->node_id = id;
          current_nodeID = id;
          robot_->get_position_by_id(id);
        }

      else
       {
          current_nodeID = "None";
          robot_->location[0] = loc[0];
          robot_->location[1] = loc[1];
        }
/*      if (!id.empty()){
        robot_->node_id = id;
        robot_->get_position_by_id(id);
      } else {
        robot_->location[0] = loc[0];
        robot_->location[1] = loc[1];
      }*/

    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(robot_->get_logger(), *robot_->get_clock(), 3000, "TF map->base_link: %s", ex.what());
    }
  }
