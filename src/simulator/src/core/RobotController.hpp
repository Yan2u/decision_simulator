#ifndef SIMULATOR_ROBOT_CONTROLLER_HPP
#define SIMULATOR_ROBOT_CONTROLLER_HPP

#include <random>
#include <thread>
#include <vector>

#include "fmt/format.h"
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include "../state/StateCollection.hpp"
#include "../state/StateDiagram.hpp"
#include "../state/StateUtils.hpp"
#include "Area.hpp"
#include "Robot.hpp"

// msg and srvs
#include "simulator/RobotAction.h"
#include "simulator/RobotCmd.h"
#include "simulator/RobotSetup.h"

namespace simulator {
class RobotController {
private:
    const int MOVE_OFFSET_X[4] = { -1, 0, 1, 0 };
    const int MOVE_OFFSET_Y[4] = { 0, -1, 0, 1 };

    Robot m_robot;

    RobotAction m_robot_action;
    bool m_acted;

    ros::Publisher m_robot_action_pub;
    ros::Subscriber m_robot_cmd_sub;
    ros::Subscriber m_robot_action_op_sub;

    std::mt19937_64 m_mt19937;
    std::uniform_int_distribution<> m_distrib;

    StateDiagram<Robot, RobotAction> m_diagram;

    int m_last_move_direction;

    bool m_is_autoplay;

    bool m_is_game_over = false;

    void robot_cmd_callback(const RobotCmd& cmd) {
        if (cmd.id != m_robot.id && cmd.id != -1) {
            return;
        }

        if (cmd.cmd == RobotCmd::BLOCKED) {
            m_robot.pos_x -= MOVE_OFFSET_X[m_last_move_direction - 4];
            m_robot.pos_y -= MOVE_OFFSET_Y[m_last_move_direction - 4];
            ROS_INFO(
                "Robot(id=%d) failed to move, back to position (%d, %d)",
                m_robot.id,
                m_robot.pos_x,
                m_robot.pos_y
            );
        } else if (cmd.cmd == RobotCmd::BUFF_ATK) {
            ROS_INFO(
                "Robot(id=%d) got BUFF_ATK(value=%.2lf), atk: %.2lf => %.2lf",
                m_robot.id,
                cmd.buff_value,
                m_robot.attack,
                m_robot.attack * (1 + cmd.buff_value)
            );
            m_robot.attack *= (1 + cmd.buff_value);
        } else if (cmd.cmd == RobotCmd::BUFF_DEF) {
            ROS_INFO(
                "Robot(id=%d) got BUFF_DEF(value=%.2lf), def: %.2lf => %.2lf",
                m_robot.id,
                cmd.buff_value,
                m_robot.defense,
                m_robot.defense * (1 + cmd.buff_value)
            );
            m_robot.defense *= (1 + cmd.buff_value);
        } else if (cmd.cmd == RobotCmd::BUFF_HEALTH) {
            ROS_INFO(
                "Robot(id=%d) got BUFF_HEALTH(value=%.2lf), health: %.2lf => %.2lf",
                m_robot.id,
                cmd.buff_value,
                m_robot.health,
                std::min(m_robot.health + cmd.buff_value, m_robot.base_health)
            );
            m_robot.health = std::min(m_robot.health + cmd.buff_value, m_robot.base_health);
        } else if (cmd.cmd == RobotCmd::BUFF_CLEAR) {
            m_robot.attack = m_robot.base_attack;
            m_robot.defense = m_robot.base_defense;
            ROS_INFO("Robot(id=%d) got BUFF_CLEAR", m_robot.id);
        } else if (cmd.cmd == RobotCmd::HARM) {
            ROS_INFO(
                "Robot(id=%d) got HARM(value=%.2lf), health: %.2lf => %.2lf",
                m_robot.id,
                cmd.harm_value,
                m_robot.health,
                m_robot.health - cmd.harm_value
            );
            m_robot.health -= cmd.harm_value;

        } else if (cmd.cmd == RobotCmd::TURN_BEGIN) {
            ROS_INFO("Robot(id=%d) got TURN_BEGIN", m_robot.id);
            m_acted = false;
        } else if (cmd.cmd == RobotCmd::GAME_OVER) {
            ROS_INFO("Robot(id=%d) got GAME_OVER", m_robot.id);
            m_is_game_over = true;
        }

        m_robot.last_cmd_types.push_back(cmd.cmd);
    }

    void robot_action_op_callback(const RobotAction& action) {
        if (m_acted) {
            ROS_INFO("Robot(id=%d) refuse to act because m_acted = true (by user)", m_robot.id);
            return;
        }

        if (action.id != -1 && action.id != m_robot.id) {
            return;
        }

        m_robot_action.action = action.action;
        m_robot_action.move_direction = action.move_direction;
        m_robot_action.turn_direction = action.turn_direction;
        m_robot_action.id = m_robot.id;

        if (m_robot_action.action == RobotAction::MOVE) {
            m_last_move_direction = m_robot_action.move_direction;
        }

        m_robot_action_pub.publish(m_robot_action);
        ROS_INFO(
            "Robot(id=%d) submitted action(type=%d) (by user)",
            m_robot.id,
            m_robot_action.action
        );
        m_acted = true;
    }

    void generate_random_action() {
        m_distrib.param(std::uniform_int_distribution<>::param_type(0, 2));
        m_robot_action.action = m_distrib(m_mt19937);
        if (m_robot_action.action == RobotAction::MOVE) {
            m_distrib.param(std::uniform_int_distribution<>::param_type(4, 7));
            m_robot_action.move_direction = m_distrib(m_mt19937);
            m_last_move_direction = m_robot_action.move_direction;
            m_robot.pos_x += MOVE_OFFSET_X[m_robot_action.move_direction - 4];
            m_robot.pos_y += MOVE_OFFSET_Y[m_robot_action.move_direction - 4];
        } else if (m_robot_action.action == RobotAction::TURN) {
            m_distrib.param(std::uniform_int_distribution<>::param_type(8, 15));
            m_robot_action.turn_direction = m_distrib(m_mt19937);
            m_robot.direction = m_robot_action.turn_direction - 8;
        }
        m_robot_action.id = m_robot.id;
    }

    void update_position() {
        static std::string node_name_full = ros::this_node::getName();
        static std::string node_name = node_name_full.substr(node_name_full.find_last_of('/') + 1);
        static std::string param_path_x = fmt::format("/simulator/{}/pos_x", node_name);
        static std::string param_path_y = fmt::format("/simulator/{}/pos_y", node_name);

        ros::param::set(param_path_x, m_robot.pos_x);
        ros::param::set(param_path_y, m_robot.pos_y);
    }

    void main_loop() {
        while (ros::ok()) {
            ros::spinOnce();

            if (m_is_game_over) {
                break;
            }

            if (!m_acted && m_is_autoplay) {
                std::string prev_state_name = m_diagram.get_current_state_name();

                m_diagram.update(m_robot);
                m_robot_action = m_diagram.take_action(m_robot);

                if (m_robot_action.action == RobotAction::MOVE) {
                    m_last_move_direction = m_robot_action.move_direction;
                    m_robot.pos_x += MOVE_OFFSET_X[m_robot_action.move_direction - 4];
                    m_robot.pos_y += MOVE_OFFSET_Y[m_robot_action.move_direction - 4];
                } else if (m_robot_action.action == RobotAction::TURN) {
                    m_robot.direction = m_robot_action.turn_direction - 8;
                }

                m_robot_action_pub.publish(m_robot_action);
                m_acted = true;
                m_robot.last_cmd_types.clear();

                ROS_INFO(
                    "Robot(id=%d) state: %s -> %s",
                    m_robot.id,
                    prev_state_name.c_str(),
                    m_diagram.get_current_state_name().c_str()
                );

                ROS_INFO(
                    "Robot(id=%d) submitted action(type=%d)",
                    m_robot.id,
                    m_robot_action.action
                );
            }

            update_position();
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    }

    void init_ros(int argc, char* argv[]) {
        ros::init(argc, argv, "Robot");

        // load parameters
        ros::NodeHandle nh("~");
        int param_camp;
        nh.getParam("camp", param_camp);
        m_robot.camp = param_camp;
        nh.getParam("id", m_robot.id);
        nh.getParam("/simulator/autoplay", m_is_autoplay);

        // setup robot
        if (!ros::service::waitForService("/simulator/robot_setup", 2000)) {
            ROS_ERROR("Failed to wait for setup service simulator_Judger: TIME OUT");
            exit(1);
        }

        ros::ServiceClient setup_server = nh.serviceClient<RobotSetup>("/simulator/robot_setup");
        RobotSetup setup;
        setup.request.camp = m_robot.camp;
        setup.request.id = m_robot.id;

        if (setup_server.call(setup)) {
            m_robot.base_attack = setup.response.attack;
            m_robot.base_defense = setup.response.defense;
            m_robot.base_health = setup.response.health;
            m_robot.health = setup.response.health;
            m_robot.attack = m_robot.base_attack;
            m_robot.defense = m_robot.base_defense;
            m_robot.pos_x = setup.response.pos_x;
            m_robot.pos_y = setup.response.pos_y;
            m_robot.direction = setup.response.direction;
        } else {
            ROS_ERROR("Failed to get response from service simulator_Judger");
            exit(1);
        }

        m_acted = true;

        std::random_device rd;
        m_mt19937 = std::mt19937_64(rd());
        m_distrib = std::uniform_int_distribution<>(1, 8);

        // init publisher and listener
        m_robot_cmd_sub =
            nh.subscribe("/simulator/robot_cmd", 1000, &RobotController::robot_cmd_callback, this);
        m_robot_action_pub = nh.advertise<RobotAction>("/simulator/robot_action", 1000);

        // init state diagram tools
        StateUtils::init();

        // init state collection
        StateCollection::init();

        // build state diagram
        m_diagram = StateDiagram<Robot, RobotAction>::from_xml(
            StateCollection::robot_actions,
            StateCollection::robot_conditions,
            "/home/yan2u/learn_ros/decision_simulator/src/simulator/state_diagram/Robot.xml"
        );

        ROS_INFO(
            "Setup diagram sucess! Current state is %s",
            m_diagram.get_current_state_name().c_str()
        );

        if (!m_is_autoplay) {
            m_robot_action_op_sub = nh.subscribe(
                "/simulator/robot_action_op",
                1000,
                &RobotController::robot_action_op_callback,
                this
            );
        }

        ros::spinOnce();

        update_position();
        main_loop();
    }

public:
    RobotController(int argc, char* argv[]) {
        init_ros(argc, argv);
    }
};
}; // namespace simulator

#endif