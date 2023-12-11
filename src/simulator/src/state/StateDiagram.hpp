#ifndef SIMULATOR_STATE_DIAGRAM_HPP
#define SIMULATOR_STATE_DIAGRAM_HPP

#include "fmt/format.h"
#include "ros/ros.h"
#include "tinyxml2.h"

#include "Condition.hpp"
#include "State.hpp"
#include "StateUtils.hpp"
#include "Transition.hpp"

namespace simulator {
template<typename Object, typename Action>
class StateDiagram {
    using ActionMap = std::map<std::string, std::function<Action(const Object&)>>;
    using ConditionMap = std::map<std::string, std::function<bool(const Object&)>>;
    using TState = State<Object, Action>;
    using TCondition = Condition<Object>;
    using TTransition = Transition<Object, Action>;
    using TStatePtr = std::shared_ptr<State<Object, Action>>;

private:
    std::map<std::string, std::vector<TTransition>> m_edges;
    std::map<std::string, TStatePtr> m_states;

    std::string m_current_state_name;

    void add_edge(
        ActionMap& action_map,
        ConditionMap& condition_map,
        const std::string& from,
        const std::string& to,
        const std::string& cond,
        int priority
    ) {
        static auto get_state = [&](const std::string& name) -> TStatePtr {
            if (action_map.find(name) == action_map.end()) {
                throw std::runtime_error(fmt::format("Action named \"{}\" not found in dict", name)
                );
            }
            if (m_states.find(name) == m_states.end()) {
                m_states[name] = TStatePtr(new TState(name, action_map[name]));
            }
            return m_states[name];
        };

        TStatePtr from_state_ptr, to_state_ptr;
        if (from == "$start_state") {
            to_state_ptr = get_state(to);
            m_current_state_name = to;
        } else {
            from_state_ptr = get_state(from);
            to_state_ptr = get_state(to);

            TransitionType type;
            TCondition condition;
            if (cond == "none") {
                type = TransitionType::NONE;
            } else if (cond == "all") {
                type = TransitionType::ALL;
            } else {
                type = TransitionType::CONDITIONAL;
                condition = StateUtils::parse_condition(condition_map, cond);
            }

            TTransition transition(type, from_state_ptr, to_state_ptr, condition, priority);
            if (m_edges.find(from) == m_edges.end()) {
                m_edges.insert({ from, {} });
            }
            m_edges[from].push_back(transition);
        }
    }

    void sort_all_transitions() {
        static auto cmp = [](TTransition& t1, TTransition& t2) -> bool { return t1 > t2; };

        for (auto&& kv: m_edges) {
            if (kv.second.size() <= 1) {
                continue;
            }
            std::sort(kv.second.begin(), kv.second.end(), cmp);
        }
    }

public:
    static StateDiagram<Object, Action>
    from_xml(ActionMap& action_map, ConditionMap& condition_map, const std::string& xml_path) {
        tinyxml2::XMLDocument document;
        tinyxml2::XMLError err = document.LoadFile(xml_path.c_str());
        if (err != tinyxml2::XMLError::XML_SUCCESS) {
            throw std::runtime_error(fmt::format(
                "Failed to load state diagram file {}: {}",
                xml_path,
                document.ErrorStr()
            ));
        }

        StateDiagram<Object, Action> diagram;

        tinyxml2::XMLElement* root = document.RootElement();
        if (strcmp(root->Name(), "StateDiagram")) {
            throw std::runtime_error(fmt::format(
                "Syntax error in state diagram file {}: Unexpected root element tag {}",
                xml_path,
                root->Name()
            ));
        }

        tinyxml2::XMLElement* child = root->FirstChildElement();
        while (child) {
            if (strcmp(child->Name(), "Edge")) {
                throw std::runtime_error(fmt::format(
                    "Syntax error in state diagram file {}: Unexpected element tag {}",
                    xml_path,
                    child->Name()
                ));
            }

            const char* attr_from;
            const char* attr_to;
            const char* attr_cond;
            int attr_priority;

            tinyxml2::XMLError err_no_attr = tinyxml2::XMLError::XML_NO_ATTRIBUTE;
            if (child->QueryStringAttribute("from", &attr_from) == err_no_attr
                || child->QueryStringAttribute("to", &attr_to) == err_no_attr
                || child->QueryStringAttribute("condition", &attr_cond) == err_no_attr)
            {
                throw std::runtime_error(fmt::format(
                    "Syntax error in state diagram file {}: Edge must have \"from\", \"to\", \"condition\" attribute",
                    xml_path
                ));
            }

            if (child->QueryIntAttribute("priority", &attr_priority) == err_no_attr) {
                attr_priority = 0;
            }

            diagram
                .add_edge(action_map, condition_map, attr_from, attr_to, attr_cond, attr_priority);
            child = child->NextSiblingElement();
        }

        diagram.sort_all_transitions();
        return diagram;
    }

    std::string get_current_state_name() {
        return m_current_state_name;
    }

    std::string debug_repr() {
        std::stringstream ss;
        ss << fmt::format("StateDiagram(n_states={}){{\n", m_states.size());
        for (auto&& kv: m_states) {
            if (m_edges.find(kv.first) == m_edges.end()) {
                continue;
            }
            auto&& transitions = m_edges[kv.first];
            for (auto&& transition: transitions) {
                ss << fmt::format("  {}\n", transition.debug_repr());
            }
        }
        ss << "}}\n";
        return ss.str();
    }

    Action take_action(const Object& object) {
        return m_states[m_current_state_name]->take_action(object);
    }

    void update(const Object& object) {
        auto&& transitions = m_edges[m_current_state_name];
        int n_transitions = transitions.size();

        auto current_state_ptr = m_states[m_current_state_name];

        for (int i = 0; i < n_transitions; ++i) {
            // ROS_INFO("Judging transitions %d: %s", i, transitions[i].debug_repr().c_str());
            if (transitions[i].can_translate(object)) {
                m_current_state_name = transitions[i].next_state_ptr()->name;
                break;
            }
        }
    }
};
}; // namespace simulator

#endif