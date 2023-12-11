#include <iostream>
#include <map>

#include "fmt/format.h"
#include "ros/ros.h"

#include "../core/RobotController.hpp"
#include "../state/StateCollection.hpp"

int main(int argc, char* argv[]) {
    simulator::RobotController controller(argc, argv);
    return 0;
}