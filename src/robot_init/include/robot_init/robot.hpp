#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <nlohmann/json.hpp>  // ? Add this!
#include "robot_init/robot_status.hpp"
#include "robot_init/robot_function.hpp"
#include "robot_init/robot_task.hpp"
#include "robot_init/robot_slam.hpp"
#include "robot_init/robot_protocol.hpp"
#include "robot_init/robot_netClient.hpp"
#include "robot_init/robot_publish.hpp"
#include "robot_init/robot_nav.hpp"
#include "robot_init/robot_error.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>  // ? ADD THIS
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>

//class STATUS;  // ? Forward declaration

class ROBOT : public rclcpp::Node, public std::enable_shared_from_this<ROBOT>
{
    friend class STATUS;
    friend class TASK;
    friend class SUBTASK;
    friend class SLAM;
    friend class ERROR_STATUS;
    friend class NavAdapter;
    
public:
    ROBOT(const nlohmann::json &robot_data);
    void register_with_server();
    void send_to_server(const nlohmann::json &msg);
    bool send_all(const uint8_t* data, size_t len);

    void listen_server();
    //std::optional<nlohmann::json> extract_data(const std::vector<uint8_t> &buffer);
    std::optional<nlohmann::json> extract_data(std::vector<uint8_t>& buffer);

    void handle_message(const nlohmann::json &msg);
    void func_login(const nlohmann::json &server_msg);
    void func_heartbeat(const nlohmann::json &server_msg);
    std::vector<uint8_t> build_charge_login_message(const std::string &id);

    void func_add_task_response(const nlohmann::json &msg);
    void func_handle_task_slice(const nlohmann::json &msg);
    void func_add_task_slice_response(const nlohmann::json &msg);
    void func_cancel_task_response(const nlohmann::json &msg);  
    void func_scanMap_task_response(const nlohmann::json &msg);    
    void func_manualMove_response(const nlohmann::json &msg); 
    std::string get_id_by_position(const std::vector<float> loc);
    void get_position_by_id(const std::string id) ;
    void func_correct_location(const nlohmann::json &msg);
    void func_free_move(const nlohmann::json &msg);
    void func_free_move_response(const nlohmann::json &msg);
    void func_correct_location_response(const nlohmann::json &msg);
    void func_manualMove(const nlohmann::json &msg);
    
    void publish_nav_goal(float x, float y, float yaw_rad);
    void publish_charge_cmd(bool charge);
    void navigateThroughWaypoints(const std::vector<nlohmann::json>& waypoints);
    void navigateThroughWaypointsForward(const std::vector<nlohmann::json>& waypoints_json);
    void navigateThroughWaypointsBackward(const std::vector<nlohmann::json>& waypoints_json);
    void rotateToYaw(float target_yaw_rad);
    
    //void publish_robot_state();
    //void publish_warning_state(std::string state);
    void publish_action(const std::string &action);
    void publish_release_scan(bool trigger);
    void publish_cancel_task(int type);
    
    void reset_state_on_disconnect();
    void robot_cancel_task();


private:

    STATUS status_;  // Add this line
    TASK task_;
    SUBTASK subtask;
    SLAM slam_;
    ERROR_STATUS error_;
    ProtocolCodec codec_;
    Bus bus_;
    NavAdapter nav_;

    std::unique_ptr<NetClient> net_;


 
    std::vector<uint8_t> rx_buffer_; // buffer cho feed codec

    int sock_fd_;
    std::thread tcp_thread_;
    std::thread listener_thread_;
    bool is_onsite;
    bool is_moving;
    bool is_task;
    bool is_slam;
    bool is_login;
    bool is_error;
    bool is_cancel;
    bool is_manual;

    std::string serial_;
    std::string id_robot_, last_register_id_ ;
    std::string device_;
    std::string ip_server_;
    std::string ip_robot;
    std::string task_state;
    std::string node_id;

    int server_port_;
    int robot_port;

    float battery = 85;
    std::vector<float> location = {0.0, 0.0};  // Or init from json
    float orientation = 0.0;
    std::string map_name = "default_map";  // Placeholder

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr suggest_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charge_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr release_scan_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr cancel_task_pub_;
    //rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr release_move_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr moveType_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;   
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav2_client_;
    
    // Ch? còn publisher cho path và subscriber cho success
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_release_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_success_;
    std::atomic<bool> nav_active_{false};


    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};
