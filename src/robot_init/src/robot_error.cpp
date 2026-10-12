// error_status.cpp
#include "robot_init/robot_error.hpp"
#include "robot_init/robot.hpp"

ERROR_STATUS::ERROR_STATUS(ROBOT* robot) : robot_(robot) {
  auto qos = rclcpp::QoS(10).best_effort();

  sub_xi_lanh_ = robot_->create_subscription<std_msgs::msg::Bool>(
    "/error/xi_lanh", qos,
    std::bind(&ERROR_STATUS::cb_xi_lanh, this, std::placeholders::_1));

  sub_battery_ = robot_->create_subscription<std_msgs::msg::Bool>(
    "/error/battery", qos,
    std::bind(&ERROR_STATUS::cb_battery, this, std::placeholders::_1));

  sub_bumper_ = robot_->create_subscription<std_msgs::msg::Bool>(
    "/error/bumper", qos,
    std::bind(&ERROR_STATUS::cb_bumper, this, std::placeholders::_1));

  //sub_emg_ = robot_->create_subscription<std_msgs::msg::Bool>(
  //  "/error/emergency_button", qos,
  //  std::bind(&ERROR_STATUS::cb_emg, this, std::placeholders::_1));

 /* sub_motor_r_ = robot_->create_subscription<robot_msgs::msg::MotorError>(
    "/error/motor_right", qos,
    std::bind(&ERROR_STATUS::cb_motor_r, this, std::placeholders::_1));

  sub_motor_l_ = robot_->create_subscription<robot_msgs::msg::MotorError>(
    "/error/motor_left", qos,
    std::bind(&ERROR_STATUS::cb_motor_l, this, std::placeholders::_1));

  sub_motors_ = robot_->create_subscription<std_msgs::msg::Int32>(
    "/error/motors", qos,
    std::bind(&ERROR_STATUS::cb_motors, this, std::placeholders::_1));*/
    
  error_pub_ = robot_->create_publisher<std_msgs::msg::Bool>("/robot/error", 10);

  check_timer_ = robot_->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&ERROR_STATUS::check_errors, this));
}

void ERROR_STATUS::cb_xi_lanh(const std_msgs::msg::Bool::SharedPtr m){ xi_lanh_err_ = m->data; }
void ERROR_STATUS::cb_battery(const std_msgs::msg::Bool::SharedPtr m){ battery_err_ = m->data; }
void ERROR_STATUS::cb_bumper(const std_msgs::msg::Bool::SharedPtr m){ bumper_err_ = m->data; }
void ERROR_STATUS::cb_emg(const std_msgs::msg::Bool::SharedPtr m){ emg_err_ = m->data; }

void ERROR_STATUS::cb_motor_r(const robot_msgs::msg::MotorError::SharedPtr m){
  motor_r_err_ = m->over_voltage || m->under_voltage || m->driver_temperature ||
                 m->motor_temperature || m->communication || m->over_current || m->others;
}
void ERROR_STATUS::cb_motor_l(const robot_msgs::msg::MotorError::SharedPtr m){
  motor_l_err_ = m->over_voltage || m->under_voltage || m->driver_temperature ||
                 m->motor_temperature || m->communication || m->over_current || m->others;
}
void ERROR_STATUS::cb_motors(const std_msgs::msg::Int32::SharedPtr m){
  motors_err_ = (m->data != 0);
}

void ERROR_STATUS::check_errors() {
  bool any = xi_lanh_err_ || battery_err_ || bumper_err_ || emg_err_
           || motor_r_err_ || motor_l_err_ || motors_err_;
  //if (any!= error_)
  //{
    error_ = any;
    
    std_msgs::msg::Bool m; m.data = any;
    error_pub_->publish(m);
    
    if(any){
       robot_->is_error = any;
       robot_->robot_cancel_task();
       //robot_->is_moving = false;
    }
    // debug log
    RCLCPP_DEBUG(robot_->get_logger(), "check_errors: is_error=%d", any);
 // }
}
