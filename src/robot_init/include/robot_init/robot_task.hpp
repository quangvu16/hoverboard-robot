// robot_task.hpp
#ifndef ROBOT_TASK_HPP
#define ROBOT_TASK_HPP

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

class ROBOT;  // Forward declaration

class TASK {
public:
    TASK(ROBOT* robot);
    void completed_task();

    std::string taskID;
    int type;
    std::string start_point;
    std::string end_point;
    bool is_completed;
    bool release_scan;
    bool release_scan_mode;

private:
    ROBOT* robot;
};

class SUBTASK {
public:
    SUBTASK(ROBOT* robot);

    void subtask_handle(const nlohmann::json &message);
    void charging(std::string cmd);
    void move();
    void end_moving();
    void lifting();
    void lowering();
    void completed_subtask();
    void take();
    void release();

    std::string task_slice_ID;
    std::string type;
    bool is_last_slice;

    std::string action_id;
    std::string action_cmd;
    int param0;
    int param3;

    std::vector<float> dest_position;
    std::string dest_point;
    float dest_angle;
    std::string current_point;
    std::vector<nlohmann::json> waypoints;
    int waypoint_type;

    std::vector<float> location_start;
    float orientation_start;
    float shelf_angle;
    bool is_completed;


private:
    ROBOT* robot;
};

#endif  // ROBOT_TASK_HPP
