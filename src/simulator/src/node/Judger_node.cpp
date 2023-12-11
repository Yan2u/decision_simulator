#include <iostream>

#include "ros/ros.h"

#include "../core/Judger.hpp"

int main(int argc, char* argv[]) {
    simulator::Judger judger(argc, argv);
    return 0;
}