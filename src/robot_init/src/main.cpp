#include <rclcpp/rclcpp.hpp>
#include "robot_init/robot.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::string config_path = "/home/ubuntu/robot_ESP_ws/src/config/robot_init/robot_config.json";
    std::ifstream file(config_path);
    json config;
    file >> config;

    auto node = std::make_shared<ROBOT>(config["robots"]);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

