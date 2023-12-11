#ifndef SIMULATOR_TRANSITION_CPP
#define SIMULATOR_TRANSITION_CPP

#include "Condition.hpp"
#include "State.hpp"

namespace simulator {

enum class TransitionType { NONE, ALL, CONDITIONAL };

template<typename Object, typename Action>
class Transition {
    using TState = State<Object, Action>;
    using TStatePtr = std::shared_ptr<State<Object, Action>>;

private:
    int m_priority;
    std::shared_ptr<TState> m_from_state_ptr;

    std::shared_ptr<TState> m_to_state_ptr;

    Condition<Object> m_condition;

public:
    TransitionType type;

    bool can_translate(const Object& object) {
        return type == TransitionType::CONDITIONAL ? m_condition.predicate(object) : true;
    }

    TStatePtr next_state_ptr() {
        return m_to_state_ptr;
    }

    bool operator<(const Transition& other) {
        if (type != other.type) {
            if (type != TransitionType::CONDITIONAL) {
                return type == TransitionType::NONE;
            }
            return other.type == TransitionType::ALL;
        }
        return m_priority < other.m_priority;
    }

    bool operator>(const Transition& other) {
        if (type != other.type) {
            if (type != TransitionType::CONDITIONAL) {
                return type == TransitionType::ALL;
            }
            return other.type == TransitionType::NONE;
        }
        return m_priority > other.m_priority;
    }

    std::string debug_repr() {
        if (type == TransitionType::ALL) {
            return fmt::format(
                "Transition(type=ALL, from={}, to={})",
                m_from_state_ptr->name,
                m_to_state_ptr->name
            );
        } else if (type == TransitionType::NONE) {
            return fmt::format(
                "Transition(type=NONE, from={}, to={})",
                m_from_state_ptr->name,
                m_to_state_ptr->name
            );
        } else if (type == TransitionType::CONDITIONAL) {
            return fmt::format(
                "Transition(type=COND, top_cond={}, from={}, to={}, priority={})",
                m_condition.debug_repr(),
                m_from_state_ptr->name,
                m_to_state_ptr->name,
                m_priority
            );
        }

        return fmt::format("Transition(unknown_type={})", (int)type);
    }

    Transition(): type(TransitionType::ALL) {}

    Transition(TransitionType type): type(type) {}

    Transition(
        TransitionType type,
        std::shared_ptr<TState> from_state_ptr,
        std::shared_ptr<TState> to_state_ptr,
        int priority
    ):
        type(type),
        m_from_state_ptr(from_state_ptr),
        m_to_state_ptr(to_state_ptr),
        m_priority(priority) {}

    Transition(
        TransitionType type,
        std::shared_ptr<TState> from_state_ptr,
        std::shared_ptr<TState> to_state_ptr,
        Condition<Object> cond,
        int priority
    ):
        type(type),
        m_from_state_ptr(from_state_ptr),
        m_to_state_ptr(to_state_ptr),
        m_condition(cond),
        m_priority(priority) {}
};
}; // namespace simulator

#endif