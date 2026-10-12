// error_status.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <robot_msgs/msg/motor_error.hpp>
#include <nlohmann/json.hpp>

class ROBOT;

class ERROR_STATUS {
public:
  explicit ERROR_STATUS(ROBOT* robot);

private:
  ROBOT* robot_;

  // các c? l?i
  bool xi_lanh_err_{false};
  bool battery_err_{false};
  bool bumper_err_{false};
  bool emg_err_{false};
  bool motor_r_err_{false};
  bool motor_l_err_{false};
  bool motors_err_{false};
  bool error_{false};

  // subs
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_xi_lanh_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_battery_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bumper_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emg_;
  rclcpp::Subscription<robot_msgs::msg::MotorError>::SharedPtr sub_motor_r_;
  rclcpp::Subscription<robot_msgs::msg::MotorError>::SharedPtr sub_motor_l_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_motors_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_pub_;
  // timer
  rclcpp::TimerBase::SharedPtr check_timer_;

  // callbacks
  void cb_xi_lanh(const std_msgs::msg::Bool::SharedPtr m);
  void cb_battery(const std_msgs::msg::Bool::SharedPtr m);
  void cb_bumper(const std_msgs::msg::Bool::SharedPtr m);
  void cb_emg(const std_msgs::msg::Bool::SharedPtr m);
  void cb_motor_r(const robot_msgs::msg::MotorError::SharedPtr m);
  void cb_motor_l(const robot_msgs::msg::MotorError::SharedPtr m);
  void cb_motors(const std_msgs::msg::Int32::SharedPtr m);

  void check_errors();  // timer callback
};
