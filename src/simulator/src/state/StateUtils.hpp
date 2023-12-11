#ifndef SIMULATOR_STATE_UTILS_HPP
#define SIMULATOR_STATE_UTILS_HPP

#include <queue>
#include <random>
#include <stack>

#include "fmt/format.h"
#include "ros/ros.h"

#include "../core/Area.hpp"
#include "../core/Robot.hpp"
#include "State.hpp"
#include "Transition.hpp"

namespace simulator {

class StateUtils {
private:
    static inline bool
    position_in(int x, int y, int xlow, int ylow, int xhigh, int yhigh, bool can_equal = true) {
        if (can_equal) {
            return x >= xlow && x <= xhigh && y >= ylow && y <= yhigh;
        } else {
            return x > xlow && x < xhigh && y > ylow && y < yhigh;
        }
    }

    static inline bool is_blocked(int x, int y) {
        if (!position_in(x, y, 0, 0, StateUtils::size_n - 1, StateUtils::size_m - 1)) {
            return true;
        }

        int n_areas = StateUtils::areas.size();
        for (int i = 0; i < n_areas; ++i) {
            auto&& area = StateUtils::areas[i];
            if (area.type == AreaType::BLOCK
                && position_in(x, y, area.x1, area.y1, area.x2, area.y2))
            {
                return true;
            }
        }

        return false;
    }

    static inline void generate_path(
        int x,
        int y,
        std::function<bool(std::tuple<int, int>)> predicate,
        std::vector<std::tuple<int, int>>& path
    ) {
        path.clear();

        // queue element: (x, y, id)
        // record element: (x, y, pre)
        // (x, y): the point (x, y), to navigate in bfs
        // id: the index of the (x, y) in the record array
        // pre: the index previous point to (x, y) in the record array
        // pre = -1 means no previous point
        std::queue<std::tuple<int, int, int>> queue;
        std::vector<std::tuple<int, int, int>> record;

        // alloc 2d array in_queue
        bool** in_queue = new bool*[StateUtils::size_n];
        for (int i = 0; i < StateUtils::size_n; ++i) {
            in_queue[i] = new bool[StateUtils::size_m]();
        }

        queue.push(std::make_tuple(x, y, 0));
        record.push_back(std::make_tuple(x, y, -1));
        in_queue[x][y] = true;
        bool found = false;
        while (!queue.empty()) {
            std::tuple<int, int, int> point = queue.front();
            queue.pop();

            // record path reversed
            if (predicate(std::make_tuple(std::get<0>(point), std::get<1>(point)))) {
                found = true;
                point = record[std::get<2>(point)];
                while (std::get<2>(point) != -1) {
                    path.push_back(std::make_tuple(std::get<0>(point), std::get<1>(point)));
                    point = record[std::get<2>(point)];
                }
                path.push_back(std::make_tuple(std::get<0>(point), std::get<1>(point)));
                std::reverse(path.begin(), path.end());
            }
            if (found) {
                break;
            }
            for (int i = 0; i < 4; ++i) {
                int new_x = std::get<0>(point) + StateUtils::MOVE_OFFSET_X[i];
                int new_y = std::get<1>(point) + StateUtils::MOVE_OFFSET_Y[i];

                if (StateUtils::is_blocked(new_x, new_y) || in_queue[new_x][new_y]) {
                    continue;
                }

                record.push_back(std::make_tuple(new_x, new_y, std::get<2>(point)));
                queue.push(std::make_tuple(new_x, new_y, record.size() - 1));
                in_queue[new_x][new_y] = true;
            }
        }

        // free 2d array in_queue
        for (int i = 0; i < StateUtils::size_n; ++i) {
            delete[] in_queue[i];
        }

        delete[] in_queue;
    }

    static int next_point_offset(
        int x,
        int y,
        std::function<bool(std::tuple<int, int>)> predicate,
        bool use_cache
    ) {
        static std::vector<std::tuple<int, int>> path_cache;

        int n_path = path_cache.size();
        int current_pos = -1;
        if (use_cache) {
            for (int i = 0; i < n_path; ++i) {
                if (std::tie(x, y) == path_cache[i]) {
                    current_pos = i;
                    break;
                }
            }
        }

        if (current_pos == -1) {
            generate_path(x, y, predicate, path_cache);

            if (path_cache.size() == 1) {
                return -1;
            } else {
                int delta_x = std::get<0>(path_cache[1]) - x;
                int delta_y = std::get<1>(path_cache[1]) - y;

                for (int i = 0; i < 4; ++i) {
                    if (delta_x == StateUtils::MOVE_OFFSET_X[i]
                        && delta_y == StateUtils::MOVE_OFFSET_Y[i])
                    {
                        return i;
                    }
                }

                throw std::runtime_error(fmt::format(
                    "Cannot match MOVE_OFFSET: (dx, dy) = ({}, {}) (current: ({}, {}))",
                    delta_x,
                    delta_y,
                    x,
                    y
                ));
            }
        } else if (current_pos == (path_cache.size() - 1)) {
            return -1;
        } else {
            int delta_x = std::get<0>(path_cache[current_pos + 1]) - x;
            int delta_y = std::get<1>(path_cache[current_pos + 1]) - y;

            for (int i = 0; i < 4; ++i) {
                if (delta_x == StateUtils::MOVE_OFFSET_X[i]
                    && delta_y == StateUtils::MOVE_OFFSET_Y[i])
                {
                    return i;
                }
            }

            throw std::runtime_error(fmt::format(
                "Cannot match MOVE_OFFSET: (dx, dy) = ({}, {}) (current: ({}, {}))",
                delta_x,
                delta_y,
                x,
                y
            ));
        }
    }

public:
    constexpr static int DIRECTION_OFFSET_X[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };
    constexpr static int DIRECTION_OFFSET_Y[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };

    constexpr static int MOVE_OFFSET_X[4] = { -1, 0, 1, 0 };
    constexpr static int MOVE_OFFSET_Y[4] = { 0, -1, 0, 1 };

    static std::vector<simulator::Area> areas;

    static int size_n;
    static int size_m;

    static inline int randint(int a, int b) {
        static std::random_device rd;
        static std::mt19937 mt19937(rd());
        static std::uniform_int_distribution<> distrib;

        if (a > b) {
            std::swap(a, b);
        }
        distrib.param(std::uniform_int_distribution<>::param_type(a, b));
        return distrib(mt19937);
    }

    static inline void strip(std::string& str) {
        str.erase(0, str.find_first_not_of(' '));
        str.erase(str.find_last_not_of(' ') + 1);
    }

    static void init() {
        XmlRpc::XmlRpcValue areas_node;
        ros::param::get("/simulator/areas", areas_node);
        int n_areas = areas_node.size();
        for (int i = 0; i < n_areas; ++i) {
            StateUtils::areas.push_back(Area(areas_node[i]));
        }

        ros::param::get("/simulator/numeric/size_n", StateUtils::size_n);
        ros::param::get("/simulator/numeric/size_m", StateUtils::size_m);
    }

    static void
    detect_enermies(const Robot& robot, std::vector<std::tuple<int, int, int>>& positions) {
        XmlRpc::XmlRpcValue node;
        ros::param::get("/simulator", node);

        std::vector<std::tuple<int, int>> all_positions;
        positions.clear();

        for (auto&& kv: node) {
            if (kv.first.find("robot") == std::string::npos) {
                continue;
            }

            if ((int)kv.second["camp"] == robot.camp) {
                continue;
            }

            all_positions.push_back(
                std::make_tuple((int)kv.second["pos_x"], (int)kv.second["pos_y"])
            );
        }

        // filter: enermies can be seen
        int n_areas = areas.size();
        int n_positions = all_positions.size();
        for (int i = 0; i < 8; ++i) {
            int new_x = robot.pos_x + StateUtils::DIRECTION_OFFSET_X[i];
            int new_y = robot.pos_y + StateUtils::DIRECTION_OFFSET_Y[i];

            while (!is_blocked(new_x, new_y)) {
                for (int j = 0; j < n_positions; ++j) {
                    if (new_x == std::get<0>(all_positions[j])
                        && new_y == std::get<1>(all_positions[j]))
                    {
                        positions.push_back(std::make_tuple(
                            std::get<0>(all_positions[j]),
                            std::get<1>(all_positions[j]),
                            i
                        ));
                    }
                }

                new_x += StateUtils::DIRECTION_OFFSET_X[i];
                new_y += StateUtils::DIRECTION_OFFSET_Y[i];
            }
        }
    }

    static bool is_in_area(const Robot& robot, AreaType type) {
        int n_areas = StateUtils::areas.size();

        for (int i = 0; i < n_areas; ++i) {
            auto&& area = StateUtils::areas[i];
            if ((area.camp == robot.camp || area.camp == 2) && area.type == type
                && StateUtils::position_in(
                    robot.pos_x,
                    robot.pos_y,
                    area.x1,
                    area.y1,
                    area.x2,
                    area.y2
                ))
            {
                return true;
            }
        }

        return false;
    }

    static bool is_in_area(int x, int y, uint8_t camp, AreaType type) {
        int n_areas = StateUtils::areas.size();

        for (int i = 0; i < n_areas; ++i) {
            auto&& area = StateUtils::areas[i];
            if ((area.camp == camp || area.camp == 2) && area.type == type
                && StateUtils::position_in(x, y, area.x1, area.y1, area.x2, area.y2))
            {
                return true;
            }
        }

        return false;
    }

    // assume that there is only one robot calling the function at all time
    // return the move offset (0 - 3) that would navigate you to the target, -1 means you are already in position
    static int navigate_to(int x, int y, uint8_t camp, AreaType type) {
        static AreaType last_type = AreaType::BLOCK;

        int offset = StateUtils::next_point_offset(
            x,
            y,
            [&](std::tuple<int, int> t) -> bool {
                return StateUtils::is_in_area(std::get<0>(t), std::get<1>(t), camp, type);
            },
            type == last_type
        );

        last_type = type;
        return offset;
    }

    // assume that there is only one robot calling the function at all time
    // return the move offset (0 - 3) that would navigate you to the target, -1 means you are already in position
    static int navigate_to(int x, int y, int target_x, int target_y) {
        static int last_target_x = -1;
        static int last_target_y = -1;

        int offset = StateUtils::next_point_offset(
            x,
            y,
            [&](std::tuple<int, int> t) -> bool { return std::tie(target_x, target_y) == t; },
            last_target_x == target_x && last_target_y == target_y
        );

        last_target_x = target_x;
        last_target_y = target_y;
        return offset;
    }

    template<typename Object>
    static Condition<Object> parse_condition(
        std::map<std::string, std::function<bool(const Object&)>>& condition_dict,
        std::string condition_syntax
    ) {
        static std::map<std::string, int> key_priority { { "NOT", 2 }, { "AND", 1 }, { "OR", 1 } };

        static std::map<std::string, ConditionType> key_type { { "NOT", ConditionType::NOT },
                                                               { "AND", ConditionType::AND },
                                                               { "OR", ConditionType::OR } };

        static std::map<std::string, bool> key_requires_two_cond { { "NOT", false },
                                                                   { "AND", true },
                                                                   { "OR", true } };

        static auto is_component_of_variable = [](char ch) -> bool {
            return ('a' <= ch && ch <= 'z') || ch == '_';
        };

        static auto is_component_of_keyword = [](char ch) -> bool {
            return 'A' <= ch && ch <= 'Z';
        };

        std::stack<std::string> stk_operator;
        std::stack<std::string> stk_suffix;
        std::stack<std::string> stk_suffix_rev;
        std::stack<Condition<Object>*> stk_expression;

        StateUtils::strip(condition_syntax);
        int len = condition_syntax.length();

        // check
        int check_parentheses = 0;
        for (int i = 0; i < len; ++i) {
            char ch = condition_syntax.at(i);
            if (ch == '(') {
                check_parentheses += 1;
            }
            if (ch == ')') {
                check_parentheses += -1;
            }
        }

        if (check_parentheses) {
            throw std::runtime_error(
                fmt::format("Parentheses does not match in syntax \"{}\"", condition_syntax)
            );
        }

        std::string current;
        for (int i = 0; i < len; ++i) {
            current.clear();
            char ch = condition_syntax.at(i);
            if (is_component_of_keyword(ch)) {
                // get logic operator
                for (; i < len; ++i) {
                    char ch2 = condition_syntax.at(i);
                    if (is_component_of_keyword(ch2)) {
                        current.push_back(ch2);
                    } else {
                        --i;
                        break;
                    }
                }
                if (key_priority.find(current) == key_priority.end()) {
                    throw std::runtime_error(fmt::format("Unexpected logic operator: {}", current));
                }
                // deal with logic operator
                int priority = key_priority[current];
                for (;;) {
                    if (stk_operator.empty() || stk_operator.top() == "(") {
                        stk_operator.push(current);
                        break;
                    } else if (priority >= key_priority[stk_operator.top()]) {
                        stk_operator.push(current);
                        break;
                    } else {
                        stk_suffix.push(stk_operator.top());
                        stk_operator.pop();
                    }
                }
            } else if (is_component_of_variable(ch)) {
                // get variable name
                for (; i < len; ++i) {
                    char ch2 = condition_syntax.at(i);
                    if (is_component_of_variable(ch2)) {
                        current.push_back(ch2);
                    } else {
                        --i;
                        break;
                    }
                }
                if (condition_dict.find(current) == condition_dict.end()) {
                    throw std::runtime_error(fmt::format("Undefined condition: {}", current));
                }
                stk_suffix.push(current);
            } else if (ch == '(') {
                stk_operator.push(std::string(1, ch));
            } else if (ch == ')') {
                while (stk_operator.top() != "(") {
                    stk_suffix.push(stk_operator.top());
                    stk_operator.pop();
                }
                std::cout << std::endl;
                stk_operator.pop();
            }
        }
        while (!stk_operator.empty()) {
            stk_suffix.push(stk_operator.top());
            stk_operator.pop();
        }
        // reverse to get expression
        while (!stk_suffix.empty()) {
            stk_suffix_rev.push(stk_suffix.top());
            stk_suffix.pop();
        }

        // build condition tree
        int total_ids = 0;
        while (!stk_suffix_rev.empty()) {
            std::string op = stk_suffix_rev.top();
            stk_suffix_rev.pop();

            if (is_component_of_variable(op.at(0))) {
                stk_expression.push(new Condition<Object>(ConditionType::SINGLE, condition_dict[op])
                );
            } else {
                // op is keyword
                ConditionType type = key_type[op];
                bool requires_two = key_requires_two_cond[op];
                if (stk_expression.size() < (requires_two ? 2 : 1)) {
                    throw std::runtime_error(fmt::format(
                        "Parse condition error: cannot get enough cond to operate (op={})",
                        op
                    ));
                }
                if (requires_two) {
                    auto cond1 = stk_expression.top();
                    stk_expression.pop();
                    auto cond2 = stk_expression.top();
                    stk_expression.pop();
                    Condition<Object>* new_cond = new Condition<Object>(
                        type,
                        nullptr,
                        std::shared_ptr<Condition<Object>>(cond1),
                        std::shared_ptr<Condition<Object>>(cond2)
                    );
                    stk_expression.push(new_cond);
                } else {
                    auto cond = stk_expression.top();
                    stk_expression.pop();
                    Condition<Object>* new_cond = new Condition<Object>(
                        type,
                        nullptr,
                        std::shared_ptr<Condition<Object>>(cond),
                        nullptr
                    );
                    stk_expression.push(new_cond);
                }
            }
        }

        assert(stk_expression.size() == 1);
        Condition<Object>* res = stk_expression.top();
        stk_expression.pop();
        return *res;
    }
};

int StateUtils::size_m = 0;
int StateUtils::size_n = 0;
std::vector<simulator::Area> StateUtils::areas {};

constexpr int StateUtils::DIRECTION_OFFSET_X[8];
constexpr int StateUtils::DIRECTION_OFFSET_Y[8];

constexpr int StateUtils::MOVE_OFFSET_X[4];
constexpr int StateUtils::MOVE_OFFSET_Y[4];

}; // namespace simulator

#endif