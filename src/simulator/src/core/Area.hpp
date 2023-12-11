#ifndef SIMULATOR_AREA_HPP
#define SIMULATOR_AREA_HPP

#include "ros/ros.h"

#include <algorithm>

namespace simulator {
enum class AreaType { BLOCK,
                      ATK,
                      DEF,
                      HEALTH };

// Rectangle area
class Area {
   public:
    AreaType type;

    // left top position
    int x1;
    int y1;

    // right bottom position
    int x2;
    int y2;

    // target camp (0 or 1)
    // 2 = all camps
    uint8_t camp;

    // buff value (health per round, atk upgrade value, etc...)
    double value;

    Area(const XmlRpc::XmlRpcValue& xml_node)
        : x1((int)xml_node["x1"]),
          y1((int)xml_node["y1"]),
          x2((int)xml_node["x2"]),
          y2((int)xml_node["y2"]),
          camp(xml_node.hasMember("camp") ? (int)xml_node["camp"] : 2),
          value(xml_node.hasMember("value") ? (double)xml_node["value"] : 0.0),
          type(AreaType((int)xml_node["type"])) {}
};
};  // namespace simulator

#endif