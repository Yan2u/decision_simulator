#ifndef SIMULATOR_STATE_HPP
#define SIMULATOR_STATE_HPP

#include <functional>
#include <string>

#include "Condition.hpp"
#include "simulator/RobotAction.h"

namespace simulator {
template<typename Object, typename Action>
class State {
private:
    std::function<Action(const Object&)> m_action;

public:
    std::string name;

    Action take_action(const Object& object) {
        return m_action(object);
    }

    State(): State("", nullptr) {}

    State(const std::string& name, std::function<Action(const Object&)> action):
        m_action(action),
        name(name) {}
};
}; // namespace simulator

#endif