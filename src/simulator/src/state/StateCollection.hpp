#ifndef SIMULATOR_STATE_COLLECTION_HPP
#define SIMULATOR_STATE_COLLECTION_HPP

#include <functional>
#include <map>

#include "ros/ros.h"

#include "../core/Robot.hpp"
#include "StateUtils.hpp"
#include "simulator/RobotAction.h"
#include "simulator/RobotCmd.h"

namespace simulator {

class StateCollection {
public:
    // single conditions
    static bool cond_see_enermy(const Robot& robot) {
        std::vector<std::tuple<int, int, int>> res;
        StateUtils::detect_enermies(robot, res);
        return !res.empty();
    }

    static bool cond_attacked(const Robot& robot) {
        int len = robot.last_cmd_types.size();
        std::stringstream ss;
        for (int i = 0; i < len; ++i) {
            if (robot.last_cmd_types[i] == RobotCmd::HARM) {
                return true;
            }
        }
        return false;
    }

    static bool cond_low_health(const Robot& robot) {
        return robot.health < 0.4 * robot.base_health;
    }

    static bool cond_enough_health(const Robot& robot) {
        return robot.health >= 0.6 * robot.base_health;
    }

    static bool cond_at_buff_health(const Robot& robot) {
        return StateUtils::is_in_area(robot, AreaType::HEALTH);
    }

    static bool cond_health_zero(const Robot& robot) {
        return robot.health <= 0;
    }

    // state actions

    static RobotAction act_attack(const Robot& robot) {
        std::vector<std::tuple<int, int, int>> res;
        StateUtils::detect_enermies(robot, res);

        RobotAction action;
        action.id = robot.id;
        if (res.empty()) {
            ROS_INFO("Robot(id=%d) act_attack but no enermies...", robot.id);

            action.action = RobotAction::FIRE;
            return action;
        }

        int x, y, d;
        std::tie(x, y, d) = res[0];
        if (d == robot.direction) {
            action.action = RobotAction::FIRE;
        } else {
            action.action = RobotAction::TURN;
            action.turn_direction = d + 8;
        }
        return action;
    }

    static RobotAction act_attacked(const Robot& robot) {
        RobotAction action;
        action.id = robot.id;
        action.action = RobotAction::MOVE;

        // move randomly to avoid attack
        action.move_direction = StateUtils::randint(0, 3) + RobotAction::MOVE_UP;

        return action;
    }

    static RobotAction act_health(const Robot& robot) {
        int offset =
            StateUtils::navigate_to(robot.pos_x, robot.pos_y, robot.camp, AreaType::HEALTH);

        RobotAction action;
        action.id = robot.id;
        if (offset == -1) {
            /// TODO: optimize action
            action.action = RobotAction::FIRE;
        } else {
            action.action = RobotAction::MOVE;
            action.move_direction = offset + 4;
        }

        return action;
    }

    static RobotAction act_stay(const Robot& robot) {
        /// TODO: use act_attack for now because act_attack does not move right now, optimize it later
        return StateCollection::act_attack(robot);
    }

    static RobotAction act_still(const Robot& robot) {
        RobotAction action;
        action.id = robot.id;

        // action = MOVE or TURN
        int r = StateUtils::randint(0, 1);
        action.action = RobotAction::MOVE + r;

        if (r) {
            // random turn_direction
            action.turn_direction = StateUtils::randint(0, 7) + RobotAction::TURN_ANGLE_0;
        } else {
            // random move_direction
            action.move_direction = StateUtils::randint(0, 3) + RobotAction::MOVE_UP;
        }

        return action;
    }

    static RobotAction act_die(const Robot& robot) {
        RobotAction action;
        action.id = robot.id;

        action.action = RobotAction::DIE;

        return action;
    }

    static std::map<std::string, std::function<bool(const Robot&)>> robot_conditions;
    static std::map<std::string, std::function<RobotAction(const Robot&)>> robot_actions;

    static void init() {
        // single conditions
        robot_conditions.insert({ "see_enermy", &StateCollection::cond_see_enermy });
        robot_conditions.insert({ "attacked", &StateCollection::cond_attacked });
        robot_conditions.insert({ "low_health", &StateCollection::cond_low_health });
        robot_conditions.insert({ "enough_health", &StateCollection::cond_enough_health });
        robot_conditions.insert({ "at_buff_health", &StateCollection::cond_at_buff_health });
        robot_conditions.insert({ "health_zero", &StateCollection::cond_health_zero });

        robot_actions.insert({ "attack", &StateCollection::act_attack });
        robot_actions.insert({ "attacked", &StateCollection::act_attacked });
        robot_actions.insert({ "health", &StateCollection::act_health });
        robot_actions.insert({ "stay", &StateCollection::act_stay });
        robot_actions.insert({ "still", &StateCollection::act_still });
        robot_actions.insert({ "die", &StateCollection::act_die });
    }
};

std::map<std::string, std::function<bool(const Robot&)>> StateCollection::robot_conditions {};
std::map<std::string, std::function<RobotAction(const Robot&)>> StateCollection::robot_actions {};

}; // namespace simulator

#endif