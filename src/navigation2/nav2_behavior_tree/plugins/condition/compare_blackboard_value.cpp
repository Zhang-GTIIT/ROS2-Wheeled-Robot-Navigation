#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <optional>
#include <variant>

using BT::NodeStatus;
using namespace std::string_literals;

class CompareBlackboardValue : public BT::ConditionNode
{
public:
  CompareBlackboardValue(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::ConditionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("key"),
             BT::InputPort<std::string>("value"),
             BT::InputPort<std::string>("operator", "==") };
  }

private:
  /* 能保存 string 或 double 即可覆盖“字符串相等”和“数字大小” */
  using Value = std::variant<std::string, double>;

  /* 从黑板上把值取出来，转成 Value；失败返回 nullopt */
  std::optional<Value> pull(const std::string& key) const
  {
    auto* bb = config().blackboard.get();

    std::string s;
    if (bb->get(key, s)) return Value{std::move(s)};

    double d = 0;
    if (bb->get(key, d)) return Value{d};

    int i = 0;
    if (bb->get(key, i)) return Value{static_cast<double>(i)};

    float f = 0;
    if (bb->get(key, f)) return Value{static_cast<double>(f)};

    return std::nullopt;
  }

  /* 把用户输入的字符串也转成 Value */
  std::optional<Value> parse(std::string_view raw, bool as_number) const
  {
    if (!as_number) return Value{std::string{raw}};
    try {
      size_t idx = 0;
      double v = std::stod(std::string{raw}, &idx);
      if (idx == raw.size()) return Value{v};
    } catch (...) {}
    return std::nullopt;
  }

  NodeStatus tick() override
  {
    std::string key, op, rhs;
    if (!getInput("key", key) || !getInput("value", rhs) || !getInput("operator", op))
      return NodeStatus::FAILURE;

    auto lhs = pull(key);
    if (!lhs) {
      RCLCPP_ERROR(rclcpp::get_logger("CompareBlackboardValue"),
                   "key '%s' not found or bad type", key.c_str());
      return NodeStatus::FAILURE;
    }

    /* 如果左边是数字，右边也按数字解析；否则按字符串 */
    bool numeric = std::holds_alternative<double>(*lhs);
    auto rhs_v   = parse(rhs, numeric);
    if (!rhs_v) {
      RCLCPP_ERROR(rclcpp::get_logger("CompareBlackboardValue"),
                   "cannot parse '%s' as %s", rhs.c_str(), numeric ? "number" : "string");
      return NodeStatus::FAILURE;
    }

    /* 字符串只支持 == != */
    if (!numeric && (op == ">" || op == "<" || op == ">=" || op == "<=")) {
      RCLCPP_ERROR(rclcpp::get_logger("CompareBlackboardValue"),
                   "operator '%s' not allowed for string", op.c_str());
      return NodeStatus::FAILURE;
    }

    /* 比较 */
    bool ok = false;
    if (numeric) {
      double l = std::get<double>(*lhs), r = std::get<double>(*rhs_v);
      if (op == "==") ok = (l == r);
      else if (op == "!=") ok = (l != r);
      else if (op == "<")  ok = (l < r);
      else if (op == "<=") ok = (l <= r);
      else if (op == ">")  ok = (l > r);
      else if (op == ">=") ok = (l >= r);
    } else {
      const std::string& l = std::get<std::string>(*lhs);
      const std::string& r = std::get<std::string>(*rhs_v);
      if (op == "==") ok = (l == r);
      else if (op == "!=") ok = (l != r);
    }
    return ok ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CompareBlackboardValue>("CompareBlackboardValue");
}