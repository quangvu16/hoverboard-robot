#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <robot_msgs/msg/battery.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class ROBOT;  // forward declaration

class STATUS {
public:
    STATUS(ROBOT* robot);

    void update_device_status();
    void update_task_status();
    void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    //void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void battery_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void taskStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void releaseNearestCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void manualCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void confirmCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void bumperCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void update_from_tf();
       
    bool in_task_state = false;
    bool last_bumper = false;
    bool ManualIntervention = false;
    std::string manual_action = "Stop";
    std::string state = "Idle";
    std::string task_state;
    std::string current_task_state;
    std::string current_nodeID;

private:
    ROBOT* robot_;
    std::thread status_thread_;
    std::thread task_thread_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr battery_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub_;  
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr confirm_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_sub_;  
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr task_release_distance_sub_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
};
