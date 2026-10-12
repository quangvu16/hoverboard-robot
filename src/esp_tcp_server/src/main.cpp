#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <nlohmann/json.hpp>

#include "esp_tcp_server/esp_node.hpp"
#include "esp_tcp_server/config.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("esp_tcp_server");
  const std::string config_path =
      pkg_share + "/config/esp_tcp_server/config.json";

  nlohmann::json cfg = esp_tcp_server::Config::load_json_file(config_path);
  auto node = std::make_shared<esp_tcp_server::EspNode>(cfg);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
