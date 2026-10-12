// robot_task.cpp
#include "robot_init/robot_task.hpp"
#include "robot_init/robot.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>

TASK::TASK(ROBOT* robot) : robot(robot), is_completed(false), release_scan(false), release_scan_mode(false) {}

void TASK::completed_task() {
    is_completed = true;
}

SUBTASK::SUBTASK(ROBOT* robot)
    : robot(robot), is_last_slice(false), is_completed(false),
     param0(0), param3(0), waypoint_type(0), shelf_angle(0.0f) {}

void SUBTASK::completed_subtask() {
    std::cout << "completed_subtask" << std::endl;
    is_completed = true;
    robot->is_moving = false;
}

void SUBTASK::charging(std::string cmd) {

    if (cmd == "DockV2")
        robot->publish_charge_cmd(true);

    else if (cmd == "Discharge")
        robot->publish_charge_cmd(false);
    else
        std::cout << "Error charge signal" << std::endl;

    //while (robot->battery < 100 && robot->is_onsite) {
    //     robot->battery++;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // }
    // if (robot->battery > 100) robot->battery = 100;
    completed_subtask();
}

// SUBTASK::move() — phiên b?n d?ng ngay khi error/cancel
// Yêu c?u: robot có các c? bool is_error, is_cancel, is_moving, is_onsite
//          và các API: navigateThroughWaypoints(vector<json>),
//                      nav_.rotate_to_yaw(double),
//                      cmd_vel_stop() ho?c cancel_navigation() n?u có.

void SUBTASK::move() {
    std::cout << "[MOVE] Start\n";
    std::cout << robot->is_error << "\n" << robot->is_cancel << "\n";
    // Thoát s?m n?u dã l?i/h?y
    if (robot->is_error || robot->is_cancel) {
        robot->is_cancel = false;
        robot->is_moving = false;
        
        return;
    }

    // Ch? chu k? di chuy?n cu k?t thúc, nhung luôn cho phép h?y ngay
    while (robot->is_moving) {
        if (robot->is_error || robot->is_cancel) {
            robot->is_cancel = false;
            robot->is_moving = false;
            std::cout << "Canceled while waiting\n";
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    //robot->is_moving = true;

    auto abort_now = [&]() -> bool {
        return robot->is_error || robot->is_cancel;
    };

    // Hai buffer tách bi?t theo move type
    std::vector<nlohmann::json> buf_fwd; // Type = 1
    std::vector<nlohmann::json> buf_rev; // Type = 2

    auto cancel_nav = [&]() {};

    auto cleanup_and_exit = [&]() {
        cancel_nav();
        buf_fwd.clear();
        buf_rev.clear();
        robot->is_moving = false;
        robot->is_cancel = false;
    };

    // Ch? nav xong nhung v?n cho phép h?y t?c th?i
    auto wait_nav_done = [&]() {
        robot->is_moving = true;
        while (robot->is_moving ) {
            if (abort_now()) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return true;
    };

    // Flush helpers
    auto flush_one = [&](std::vector<nlohmann::json>& buf, bool is_forward) -> bool {
        if (buf.empty()) return true;
        if (abort_now()) { buf.clear(); return false; }

        std::cout << "[DEBUG] Flush " << buf.size()
                  << " waypoints (" << (is_forward ? "forward" : "backward") << ")\n";

        if (is_forward) {
            robot->navigateThroughWaypointsForward(buf);   // TODO: hi?n th?c ho?c d?i tên theo l?p c?a b?n
        } else {
            robot->navigateThroughWaypointsBackward(buf);  // TODO: hi?n th?c ho?c d?i tên theo l?p c?a b?n
        }
        buf.clear();

        if (!wait_nav_done()) return false;
        return true;
    };

    auto flush_all = [&]() -> bool {
        if (!flush_one(buf_fwd, true))  return false;
        if (!flush_one(buf_rev, false)) return false;
        return true;
    };

    // Duy?t waypoint
    for (size_t i = 0; i < waypoints.size(); ++i) {
        if (abort_now()) { cleanup_and_exit(); return; }

        const auto& wp = waypoints[i];
        const int type = wp.value("Type", 1);

        // Gom theo move type
        if (type == 1) {
            // Tru?c khi d?i sang forward, d?m b?o không còn backlog backward
            if (!buf_rev.empty()) {
                if (!flush_one(buf_rev, false)) { cleanup_and_exit(); return; }
            }
            buf_fwd.push_back(wp);
            continue;
        }
        if (type == 2) {
            // Tru?c khi d?i sang backward, d?m b?o không còn backlog forward
            if (!buf_fwd.empty()) {
                if (!flush_one(buf_fwd, true)) { cleanup_and_exit(); return; }
            }
            buf_rev.push_back(wp);
            continue;
        }

        // Type 6: có logic cua r?ng. M?c d?nh coi là di ti?n (gi? nguyên nhu hi?n t?i).
        if (type == 6 && i > 0) {
            const auto& prev = waypoints[i - 1];
            std::vector<int> p0 = prev.value("Position", std::vector<int>{0, 0});
            std::vector<int> p1 = wp.value("Position",   std::vector<int>{0, 0});

            const double x0 = p0[0] / 1000.0, y0 = p0[1] / 1000.0;
            const double x1 = p1[0] / 1000.0, y1 = p1[1] / 1000.0;
            double dx = x1 - x0, dy = y1 - y0;
            double length = std::hypot(dx, dy);
            if (length == 0.0) length = 1.0;

            const double ratio = 0.3;
            const double px = x1 - dx * ratio;
            const double py = y1 - dy * ratio;

            const double offset_strength = 0.3;
            const double ox = -dy / length * offset_strength;
            const double oy =  dx / length * offset_strength;
            const double cx = px + ox, cy = py + oy;

            nlohmann::json injected_wp = wp;
            injected_wp["Position"] = {
                static_cast<int>(cx * 1000),
                static_cast<int>(cy * 1000)
            };
            injected_wp["Type"] = 1;
            injected_wp["DestAngle"] = wp.value("DestAngle", 0);
            injected_wp["LinearVelocity"] = wp.value("LinearVelocity", 1500);

            std::cout << "[MOVE] Add wide-turn helper at (" << injected_wp["Position"][0]
                      << ", " << injected_wp["Position"][1] << ") before Type 6\n";

            // N?u mu?n chèn di?m cua r?ng thì b?t dòng sau:
            // buf_fwd.push_back(injected_wp);

            // Hi?n t?i d?y waypoint g?c, m?c d?nh forward
            buf_fwd.push_back(wp);
            continue;
        }

        // G?p type d?c bi?t: flush tru?c r?i x? lý
        if (!flush_all()) { cleanup_and_exit(); return; }
        if (abort_now()) { cleanup_and_exit(); return; }

        if (type == 3 || type == 4) {
            float dest_angle_deg = wp.value("DestAngle", 0.0f) / 1000.0f;
            float dest_angle_rad =
                std::fmod(dest_angle_deg * static_cast<float>(M_PI) / 180.0f + static_cast<float>(M_PI),
                          2.0f * static_cast<float>(M_PI)) - static_cast<float>(M_PI);

            std::cout << "[MOVE] Rotate in place to " << dest_angle_deg << " deg\n";
            if (!abort_now()) robot->nav_.rotate_to_yaw(dest_angle_rad);
            if (abort_now()) { cleanup_and_exit(); return; }
            continue;
        }

        if (type == 5) {
            std::cout << "[MOVE] Wait 1s\n";
            for (int t = 0; t < 100; ++t) {
                if (abort_now()) { cleanup_and_exit(); return; }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            continue;
        }

        std::cout << "[MOVE] Unsupported Type: " << type << " — skip\n";
    }

    // K?t thúc vòng
    if (abort_now()) { cleanup_and_exit(); return; }

    // Flush ph?n còn l?i
    if (!flush_all()) { cleanup_and_exit(); return; }
    if (abort_now()) { cleanup_and_exit(); return; }
    
    auto wrap_pi = [](double a)->double {
    a = std::fmod(a + M_PI, 2.0*M_PI);
    if (a < 0) a += 2.0*M_PI;
    return a - M_PI;
};

if (is_last_slice && !abort_now()) {
    double target_yaw_rad = wrap_pi(static_cast<double>(dest_angle) * M_PI / 180.0);
    std::cout << "[MOVE] Final yaw align to " << (target_yaw_rad * 180.0 / M_PI) << " deg\n";
    robot->nav_.rotate_to_yaw(target_yaw_rad);
    if (abort_now()) { cleanup_and_exit(); return; }
}


    // Hoàn t?t
    std::cout << "[MOVE] Done\n";
    completed_subtask();

}



void SUBTASK::end_moving() {
    if (robot->location == dest_position) {
        completed_subtask();
    }
}

void SUBTASK::lifting() {
    std::cout << robot->device_ << " lifting shelf..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    std::cout << robot->device_ << " shelf lifted." << std::endl;
    completed_subtask();
}

void SUBTASK::lowering() {
    std::cout << robot->device_ << " lowering shelf..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    std::cout << robot->device_ << " shelf lowered." << std::endl;
    completed_subtask();
}

void SUBTASK::take() {
     robot->publish_action("Take");
}

void SUBTASK::release() {
     robot->publish_action("Release");
}

void SUBTASK::subtask_handle(const nlohmann::json &message) {
    std::cout << "[DEBUG] âž¤ SUBTASK::subtask_handle entered" << std::endl;
    std::cout << "[DEBUG] Full JSON Message:\n" << message.dump(2) << std::endl;

    const auto& task_slice = message.at("Body").at("TaskSlice");
    std::cout << "[DEBUG] task_slice content:\n" << task_slice.dump(2) << std::endl;

    is_last_slice = task_slice.at("IsLastSlice").get<bool>();
    current_point = task_slice.at("CurrentPoint").get<std::string>();
    robot->task_.taskID = message.at("Body").at("TaskID").get<std::string>();
    task_slice_ID = task_slice.at("ID").get<std::string>();

    std::cout << "[DEBUG] TaskSlice ID: " << task_slice_ID << std::endl;
    std::cout << "[DEBUG] Task ID: " << robot->task_.taskID << std::endl;
    std::cout << "[DEBUG] Is Last Slice: " << is_last_slice << std::endl;

    auto actions = task_slice.value("Actions", nlohmann::json::array());
    bool actions_empty = actions.is_array() && std::all_of(actions.begin(), actions.end(), [](const auto& act) {
        return act.value("Action", nlohmann::json::array()).empty();
    });
    std::cout << "[DEBUG] Actions empty: " << actions_empty << std::endl;

    waypoints = task_slice.at("WayPoints").get<std::vector<nlohmann::json>>();
    bool waypoints_empty = waypoints.empty();
    std::cout << "[DEBUG] Waypoints empty: " << waypoints_empty << std::endl;

    is_completed = false;
    if (actions_empty && !waypoints_empty) {
        std::cout << "[DEBUG] Executing waypoint-based movement" << std::endl;
        const auto& first_wp = waypoints.front();
        const auto& last_wp = waypoints.back();

        waypoint_type = last_wp["Type"].get<int>();
        dest_position = last_wp.at("Position").get<std::vector<float>>();
        dest_point = last_wp["DestPoint"].get<std::string>();
        dest_angle = last_wp["DestAngle"].get<float>() / 1000.0f;

        std::cout << "[DEBUG] DestPoint: " << dest_point << ", DestAngle: " << dest_angle << std::endl;
        std::cout << "[DEBUG] Launching move thread..." << std::endl;
        std::thread(&SUBTASK::move, this).detach();

    } else if (!actions_empty && waypoints_empty) {
        std::cout << "[DEBUG] Executing action-based subtask" << std::endl;
        for (const auto& action : actions) {
            for (const auto& sub_action : action.value("Action", nlohmann::json::array())) {
                std::string cmd = sub_action.value("Cmd", "");
                int param0 = sub_action.value("Param0", 0);
                std::cout << "[DEBUG] Action Cmd: " << cmd << ", Param0: " << param0 << std::endl;
                if (cmd == "Lift") {
                    if (param0 == 1) std::thread(&SUBTASK::lifting, this).detach();
                    else if (param0 == 2) std::thread(&SUBTASK::lowering, this).detach();
                } else if (cmd == "DockV2" || cmd == "Discharge") {
                    charging(cmd);
                }
                else if (cmd == "Take" ) {
                    std::thread(&SUBTASK::take, this).detach();
                }
                else if (cmd == "Release" ) {
                    std::thread(&SUBTASK::release, this).detach();
                }
            }
        }
    } else {
        std::cerr << "[DEBUG] âŒ Invalid subtask format: neither pure movement nor pure action." << std::endl;
    }

    while (!is_completed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (is_last_slice) {
        std::cout << "[DEBUG] âœ… Task slice is last. Marking task as complete." << std::endl;
/*        nlohmann::json task_status = {
            {"desc", "TASK SUCCESS"},
            {"id", robot->task_.taskID},
            {"time", static_cast<int>(time(nullptr))}
        };*/
        robot->status_.task_state = "Finished";
        robot->status_.state = "Idle";
        //std::cout << robot->status_.task_state << std::endl;
    } else {
        std::cout << "[DEBUG] âœ… Subtask complete (not last slice)." << std::endl;
        nlohmann::json subtask_status = {
            {"desc", "SUBTASK SUCCESS"},
            {"id", task_slice_ID},
            {"time", static_cast<int>(time(nullptr))}
        };
        robot->status_.task_state = "Finished";
        robot->status_.state = "InTask";
    }
}

