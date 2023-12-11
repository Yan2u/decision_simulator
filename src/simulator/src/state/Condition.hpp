#ifndef SIMULATOR_CONDITION_HPP
#define SIMULATOR_CONDITION_HPP

#include <functional>

namespace simulator {

enum class ConditionType { SINGLE,
                           OR,
                           AND,
                           NOT };

template <typename Object>
class Condition {
   public:
    std::shared_ptr<Condition<Object>> lchild;
    std::shared_ptr<Condition<Object>> rchild;
    std::function<bool(const Object&)> predicate_func;
    ConditionType type;

    bool predicate(const Object& object) {
        if (type == ConditionType::SINGLE) {
            return predicate_func == nullptr ? true : predicate_func(object);
        } else if (type == ConditionType::NOT) {
            return lchild ? !(lchild->predicate(object)) : true;
        } else if (type == ConditionType::OR) {
            if (lchild == nullptr && rchild == nullptr) {
                return true;
            }
            if (lchild == nullptr || rchild == nullptr) {
                return lchild == nullptr ? rchild->predicate(object)
                                         : lchild->predicate(object);
            }
            return rchild->predicate(object) || lchild->predicate(object);
        } else if (type == ConditionType::AND) {
            if (lchild == nullptr && rchild == nullptr) {
                return true;
            }
            if (lchild == nullptr || rchild == nullptr) {
                return lchild == nullptr ? rchild->predicate(object)
                                         : lchild->predicate(object);
            }
            return rchild->predicate(object) && lchild->predicate(object);
        }

        throw std::runtime_error(fmt::format("Unexpected condition type: {}", (int)type));
    }

    std::string debug_repr() {
        if (type == ConditionType::SINGLE) {
            return fmt::format("Condition(type=SINGLE)");
        } else if (type == ConditionType::AND) {
            return fmt::format("Condition(type=AND)");
        } else if (type == ConditionType::OR) {
            return fmt::format("Condition(type=OR)");
        } else if (type == ConditionType::NOT) {
            return fmt::format("Condition(type=NOT)");
        }

        return fmt::format("Condition(unknown_type={})", (int)type);
    }

    Condition()
        : Condition(ConditionType::SINGLE, nullptr, nullptr, nullptr) {}

    Condition(ConditionType type, std::function<bool(const Object&)> predicate)
        : Condition(type, predicate, nullptr, nullptr) {}

    Condition(
        ConditionType type,
        std::function<bool(const Object&)> predicate,
        std::shared_ptr<Condition<Object>> lchild,
        std::shared_ptr<Condition<Object>> rchild)
        : type(type),
          predicate_func(predicate),
          lchild(lchild),
          rchild(rchild) {}
};

};  // namespace simulator

#endif