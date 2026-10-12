#pragma once
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <optional>


class RobotFunction {
public:
    static std::string get_id_by_position(const std::vector<float>& pos );
    static std::optional<std::pair<float, float>> get_position_by_id(const std::string& node_id);

private:
    static std::string json_path;

};
