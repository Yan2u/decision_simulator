#ifndef SIMULATOR_JUDGER_HPP
#define SIMULATOR_JUDGER_HPP

#include <thread>
#include <vector>

#include "fmt/format.h"
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include "Area.hpp"
#include "Board.hpp"
#include "Robot.hpp"

// msg and srvs
#include "simulator/RobotAction.h"
#include "simulator/RobotCmd.h"
#include "simulator/RobotSetup.h"
#include "simulator/RobotView.h"

namespace simulator {
class Judger {
private:
    const int DIRECTION_OFFSET_X[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };
    const int DIRECTION_OFFSET_Y[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };

    const int MOVE_OFFSET_X[4] = { -1, 0, 1, 0 };
    const int MOVE_OFFSET_Y[4] = { 0, -1, 0, 1 };

    int m_n_players;
    int m_size_m, m_size_n;
    double m_base_health, m_base_attack, m_base_defense;
    std::vector<Area> m_areas;
    std::vector<Robot> m_robots;
    std::vector<RobotAction> m_robot_actions;

    ros::Publisher m_robot_cmd_pub;
    ros::Subscriber m_robot_action_sub;
    ros::Subscriber m_robot_cmd_op_sub;

    Board m_board;

    // count of registered robots in camp 0 (BLUE) and 1 (RED)
    int m_n_registered_0, m_n_registered_1;

    // count of robots that already acted in this round
    int m_n_acted_robots;

    int m_n_turns;

    bool m_is_autoplay;

    bool m_is_game_over = false;

    static inline bool
    position_in(int x, int y, int xlow, int ylow, int xhigh, int yhigh, bool can_equal = true) {
        if (can_equal) {
            return x >= xlow && x <= xhigh && y >= ylow && y <= yhigh;
        } else {
            return x > xlow && x < xhigh && y > ylow && y < yhigh;
        }
    }

    // response to robot setup
    bool setup_robot_response(RobotSetup::Request& req, RobotSetup::Response& resp) {
        if (req.camp == req.BLUE) {
            ++m_n_registered_0;

            /// TODO: giving initial position more reasonably
            resp.pos_x = (m_size_m - 1) / 4;
            resp.pos_y = (m_size_n - 1) / 2;
            resp.direction = 6;
        } else {
            ++m_n_registered_1;

            /// TODO: giving initial position more reasonably
            resp.pos_x = (m_size_m - 1) / 4 * 3;
            resp.pos_y = (m_size_n - 1) / 2;
            resp.direction = 2;
        }

        resp.health = m_base_health;
        resp.attack = m_base_attack;
        resp.defense = m_base_defense;

        Robot robot = Robot();
        robot.pos_x = resp.pos_x;
        robot.pos_y = resp.pos_y;
        robot.base_health = resp.health;
        robot.base_attack = resp.health;
        robot.base_defense = resp.defense;
        robot.health = resp.health;
        robot.attack = resp.attack;
        robot.defense = resp.defense;
        robot.direction = resp.direction;
        robot.camp = req.camp;
        robot.id = req.id;
        m_robots.push_back(robot);

        ROS_INFO(
            "Robot(camp=%d, pos=(%d, %d), hp=%.2lf, atk=%.2lf, def=%.2lf)",
            robot.camp,
            robot.pos_x,
            robot.pos_y,
            robot.health,
            robot.attack,
            robot.defense
        );

        return true;
    }

    void move_robot(Robot& robot, const RobotAction& action) {
        int new_x = robot.pos_x + MOVE_OFFSET_X[action.move_direction - 4];
        int new_y = robot.pos_y + MOVE_OFFSET_Y[action.move_direction - 4];

        int n_areas = m_areas.size();
        bool blocked = false;

        for (int i = 0; i < n_areas; ++i) {
            if (m_areas[i].type == AreaType::BLOCK
                && position_in(
                    new_x,
                    new_y,
                    m_areas[i].x1,
                    m_areas[i].y1,
                    m_areas[i].x2,
                    m_areas[i].y2
                ))
            {
                blocked = true;
                break;
            }
        }

        for (int i = 0; i < m_n_players; ++i) {
            if (m_robots[i].pos_x == new_x && m_robots[i].pos_y == new_y) {
                blocked = true;
                break;
            }
        }

        if (!blocked && position_in(new_x, new_y, 0, 0, m_size_n - 1, m_size_m - 1)) {
            ROS_INFO(
                "Robot(id=%d) moved from (%d, %d) to (%d, %d)",
                robot.id,
                robot.pos_x,
                robot.pos_y,
                new_x,
                new_y
            );
            robot.pos_x = new_x;
            robot.pos_y = new_y;
        } else {
            RobotCmd cmd;
            cmd.cmd = RobotCmd::BLOCKED;
            cmd.id = robot.id;
            m_robot_cmd_pub.publish(cmd);

            ROS_INFO(
                "Robot(id=%d) tried to moved from (%d, %d) to (%d, %d) but blocked",
                robot.id,
                robot.pos_x,
                robot.pos_y,
                new_x,
                new_y
            );
        }
    }

    void fire_robot(Robot& robot, const RobotAction& action) {
        ROS_INFO("Robot(id=%d) fired", robot.id);

        int x = robot.pos_x + DIRECTION_OFFSET_X[robot.direction],
            y = robot.pos_y + DIRECTION_OFFSET_Y[robot.direction];
        int n_areas = m_areas.size();
        bool block = false;
        while (x >= 0 && x < m_size_n && y >= 0 && y < m_size_m) {
            for (int i = 0; i < n_areas; ++i) {
                if (m_areas[i].type == AreaType::BLOCK
                    && position_in(
                        x,
                        y,
                        m_areas[i].x1,
                        m_areas[i].y1,
                        m_areas[i].x2,
                        m_areas[i].y2
                    ))
                {
                    block = true;
                    break;
                }
            }

            if (block) {
                break;
            }

            for (int i = 0; i < m_n_players; ++i) {
                if (m_robots[i].pos_x == x && m_robots[i].pos_y == y
                    && m_robots[i].camp != robot.camp)
                {
                    // harm and send messages
                    RobotCmd cmd;
                    cmd.cmd = RobotCmd::HARM;
                    cmd.harm_value = std::max(robot.attack - m_robots[i].defense, 0.0);
                    cmd.id = m_robots[i].id;
                    m_robot_cmd_pub.publish(cmd);

                    m_robots[i].health -= cmd.harm_value;
                    ROS_INFO(
                        "Robot(id=%d) shot by Robot(id=%d) and got %.2lf damage",
                        m_robots[i].id,
                        robot.id,
                        cmd.harm_value
                    );
                }
            }

            x = x + DIRECTION_OFFSET_X[robot.direction];
            y = y + DIRECTION_OFFSET_Y[robot.direction];
        }
    }

    void buff_robot(Robot& robot) {
        int n_areas = m_areas.size();

        for (int i = 0; i < n_areas; ++i) {
            if (position_in(
                    robot.pos_x,
                    robot.pos_y,
                    m_areas[i].x1,
                    m_areas[i].y1,
                    m_areas[i].x2,
                    m_areas[i].y2
                )
                && m_areas[i].camp == robot.camp)
            {
                RobotCmd cmd;
                cmd.buff_value = m_areas[i].value;
                cmd.id = robot.id;
                if (m_areas[i].type == AreaType::ATK) {
                    cmd.cmd = RobotCmd::BUFF_ATK;
                    robot.attack = robot.attack * (1.0 + m_areas[i].value);
                    ROS_INFO("Robot(id=%d) got BUFF_ATK(value=%.2lf)", robot.id, m_areas[i].value);
                } else if (m_areas[i].type == AreaType::DEF) {
                    cmd.cmd = RobotCmd::BUFF_DEF;
                    robot.defense = robot.defense * (1.0 + m_areas[i].value);
                    ROS_INFO("Robot(id=%d) got BUFF_DEF(value=%.2lf)", robot.id, m_areas[i].value);
                } else if (m_areas[i].type == AreaType::HEALTH) {
                    cmd.cmd = RobotCmd::BUFF_HEALTH;
                    robot.health = std::min(robot.health + m_areas[i].value, robot.base_health);
                    ROS_INFO(
                        "Robot(id=%d) got BUFF_HEALTH(value=%.2lf)",
                        robot.id,
                        m_areas[i].value
                    );
                }
                m_robot_cmd_pub.publish(cmd);
            }
        }
    }

    void robot_action_callback(const RobotAction& action) {
        ++m_n_acted_robots;
        m_robot_actions.push_back(action);
    }

    void robot_cmd_op_callback(const RobotCmd& cmd) {
        m_robot_cmd_pub.publish(cmd);
        ROS_INFO(
            "published command(type=%d, buff_val=%.2lf, harm_val=%.2lf)",
            cmd.cmd,
            cmd.buff_value,
            cmd.harm_value
        );
    }

    void summary_robot_actions() {
        static auto cmp = [](const RobotAction& a, const RobotAction& b) -> bool {
            return a.action < b.action;
        };

        std::sort(m_robot_actions.begin(), m_robot_actions.end(), cmp);

        // summary - actions
        for (int i = 0; i < m_n_players; ++i) {
            int robot_idx = -1;
            for (int j = 0; j < m_n_players; ++j) {
                if (m_robots[j].id == m_robot_actions[i].id) {
                    robot_idx = j;
                    break;
                }
            }

            if (robot_idx >= 0) {
                if (m_robot_actions[i].action == RobotAction::FIRE) {
                    fire_robot(m_robots[robot_idx], m_robot_actions[i]);
                } else if (m_robot_actions[i].action == RobotAction::MOVE) {
                    move_robot(m_robots[robot_idx], m_robot_actions[i]);
                } else if (m_robot_actions[i].action == RobotAction::TURN) {
                    auto&& robot = m_robots[robot_idx];
                    robot.direction = m_robot_actions[i].turn_direction - 8;

                    ROS_INFO("Robot(id=%d) turned, new direction = %d", robot.id, robot.direction);
                } else if (m_robot_actions[i].action == RobotAction::DIE) {
                    m_is_game_over = true;
                    return;
                }
            } else {
                ROS_INFO("Got unexpected Robot(id=%d) (not recorded)", m_robot_actions[i].id);
                exit(1);
            }
        }

        RobotCmd cmd;
        cmd.cmd = RobotCmd::BUFF_CLEAR;
        cmd.id = -1;
        m_robot_cmd_pub.publish(cmd);

        m_board.show();

        for (int i = 0; i < m_n_players; ++i) {
            m_robots[i].attack = m_base_attack;
            m_robots[i].defense = m_base_defense;
            buff_robot(m_robots[i]);
        }
    }

    void summary_game_over() {
        double camp_hp_0 = 0.0;
        double camp_hp_1 = 0.0;
        for (int i = 0; i < m_n_players; ++i) {
            if (m_robots[i].camp == 0u) {
                camp_hp_0 += m_robots[i].health;
            } else {
                camp_hp_1 += m_robots[i].health;
            }
        }
        if (camp_hp_0 > camp_hp_1) {
            ROS_INFO("[GAME OVER] Winner: CAMP 0");
        } else if (camp_hp_0 == camp_hp_1) {
            ROS_INFO("[GAME OVER] Winner: CAMP 0 and CAMP 1");
        } else {
            ROS_INFO("[GAME OVER] Winner: CAMP 1");
        }

        RobotCmd cmd;
        cmd.id = -1;
        cmd.cmd = RobotCmd::GAME_OVER;
        m_robot_cmd_pub.publish(cmd);
    }

    void main_loop() {
        while (ros::ok()) {
            if (m_n_acted_robots == m_n_players) {
                // summary and start next round
                summary_robot_actions();
                m_robot_actions.clear();
                m_n_acted_robots = 0;

                if (m_is_game_over) {
                    summary_game_over();
                    ROS_INFO("Waiting for all players to exit...");
                    while (m_robot_cmd_pub.getNumSubscribers() > 0) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    break;
                }

                RobotCmd cmd;
                cmd.cmd = RobotCmd::TURN_BEGIN;
                cmd.id = -1;
                m_robot_cmd_pub.publish(cmd);

                ++m_n_turns;

                ROS_INFO("Turn finished, entering new turn (n_turns=%d)", m_n_turns);
            }

            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // init ros
    void init_ros(int argc, char* argv[]) {
        ros::init(argc, argv, "Judger");

        // load parameters
        ros::param::get("/simulator/numeric/n_players", m_n_players);
        ros::param::get("/simulator/numeric/size_n", m_size_n);
        ros::param::get("/simulator/numeric/size_m", m_size_m);
        ros::param::get("/simulator/numeric/health", m_base_health);
        ros::param::get("/simulator/numeric/attack", m_base_attack);
        ros::param::get("/simulator/numeric/defense", m_base_defense);
        ros::param::get("/simulator/autoplay", m_is_autoplay);

        if (m_is_autoplay) {
            ROS_INFO("AUTOPLAY is ON");
        } else {
            ROS_INFO("AUTOPLAY is OFF");
        }

        // load areas
        XmlRpc::XmlRpcValue areas;
        ros::param::get("/simulator/areas", areas);
        int n_areas = areas.size();
        for (int i = 0; i < n_areas; ++i) {
            m_areas.push_back(Area(areas[i]));
        }

        m_board = Board(m_size_n, m_size_m, 800, 800, &m_robots, &m_areas);

        // setup server
        ros::NodeHandle nh;
        ros::ServiceServer server =
            nh.advertiseService("/simulator/robot_setup", &Judger::setup_robot_response, this);

        m_n_registered_0 = 0;
        m_n_registered_1 = 0;
        m_n_acted_robots = 0;
        m_n_turns = 0;

        m_robot_actions.reserve(m_n_players);

        ROS_INFO("Waiting for %d robot nodes to connect...", m_n_players);

        while (m_n_registered_0 + m_n_registered_1 < m_n_players) {
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if (!ros::ok()) {
                exit(1);
            }
        }

        ROS_INFO("%d robot nodes successfully connected...", m_n_players);

        // setup topic publisher and topic listener
        m_robot_cmd_pub = nh.advertise<RobotCmd>("/simulator/robot_cmd", 1000);
        m_robot_action_sub =
            nh.subscribe("/simulator/robot_action", 1000, &Judger::robot_action_callback, this);

        if (!m_is_autoplay) {
            m_robot_cmd_op_sub =
                nh.subscribe("/simulator/robot_cmd_op", 1000, &Judger::robot_cmd_op_callback, this);
        }

        ROS_INFO("Waiting for robot_cmd subscribers...");

        while (m_robot_cmd_pub.getNumSubscribers() < m_n_players) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        ROS_INFO("%d robot_cmd subscribers connected...", m_n_players);

        for (int i = 0; i < m_n_players; ++i) {
            buff_robot(m_robots[i]);
        }

        RobotCmd cmd;
        cmd.cmd = RobotCmd::TURN_BEGIN;
        cmd.id = -1;
        m_robot_cmd_pub.publish(cmd);

        ROS_INFO("Entering game...");
        ++m_n_turns;
        main_loop();
    }

public:
    Judger() {}

    Judger(int argc, char* argv[]) {
        init_ros(argc, argv);
    }
};
}; // namespace simulator
#endif