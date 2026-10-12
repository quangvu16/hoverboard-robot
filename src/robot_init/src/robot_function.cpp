#include "robot_init/robot_function.hpp"
#include <fstream>
#include <cmath>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>

std::string RobotFunction::json_path = "/home/ubuntu/robot_slam_ws/src/config/robot_init/topo.json";

std::string RobotFunction::get_id_by_position(const std::vector<float>& pos) {
    std::ifstream file(json_path);
    if (!file.is_open()) {
        std::cerr << "❌ Failed to open " << json_path << "\n";
        return "";
    }

    nlohmann::json data;
    try {
        file >> data;
    } catch (const std::exception& e) {
        std::cerr << "❌ JSON parse error: " << e.what() << "\n";
        return "";
    }

    if (!data.contains("nodes")) {
        std::cerr << "❌ Missing 'nodes' field in JSON\n";
        return "";
    }

    const float window_size = 500.0f;
    const float half_window = window_size / 2.0f;

    for (const auto& node : data["nodes"]) {
        float x = node["coordinate"]["x"];
        float y = node["coordinate"]["y"];
        if (std::abs(pos[0] - x) <= half_window && std::abs(pos[1] - y) <= half_window) {
            return node["id"];
        }
    }

    return "";  // No matching node found
}

std::optional<std::pair<float, float>> RobotFunction::get_position_by_id(const std::string& node_id) {
    std::ifstream file(json_path);
    if (!file.is_open()) {
        std::cerr << "❌ Failed to open " << json_path << "\n";
        return std::nullopt;
    }

    nlohmann::json data;
    try {
        file >> data;
    } catch (const std::exception& e) {
        std::cerr << "❌ JSON parse error: " << e.what() << "\n";
        return std::nullopt;
    }

    if (!data.contains("nodes")) {
        std::cerr << "❌ Missing 'nodes' field in JSON\n";
        return std::nullopt;
    }

    for (const auto& node : data["nodes"]) {
        if (node["id"] == node_id) {
            float x = node["coordinate"]["x"];
            float y = node["coordinate"]["y"];
            return std::make_pair(x, y);
        }
    }

    return std::nullopt;  // ID not found
}
