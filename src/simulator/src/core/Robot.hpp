#ifndef SIMULATOR_ROBOT_HPP
#define SIMULATOR_ROBOT_HPP

#include "simulator/RobotCmd.h"

#include <vector>

namespace simulator {
class Robot {
public:
    double health;

    double base_attack;
    double base_defense;
    double base_health;

    double attack;
    double defense;

    uint8_t camp;

    uint8_t direction;
    int pos_x;
    int pos_y;
    int id;

    std::vector<uint8_t> last_cmd_types;
};
}; // namespace simulator

#endif