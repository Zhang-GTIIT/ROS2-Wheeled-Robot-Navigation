#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <sstream>
#include <iomanip>
#include <algorithm>

class DumpBlackboard : public BT::SyncActionNode
{
public:
  DumpBlackboard(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("dump_blackboard_node");
  }

  static BT::PortsList providedPorts()
  {
    // 支持单个键名或多个键名
    return { 
      BT::InputPort<std::string>("key_name"),
      BT::InputPort<std::string>("key_names")  // 改为字符串类型，支持多种输入格式
    };
  }

  BT::NodeStatus tick() override
  {
    auto blackboard = config().blackboard;
    std::vector<std::string> keys_to_search;
    
    // 获取要查找的键名（支持单个或多个）
    std::string key_names_str;
    std::string single_key_name;
    
    // 获取输入
    auto result_multiple = getInput<std::string>("key_names", key_names_str);
    auto result_single = getInput<std::string>("key_name", single_key_name);
    
    // 处理输入逻辑：支持单个或多个输入，任何一个有效即可
    bool has_single = (result_single && !single_key_name.empty());
    bool has_multiple = (result_multiple && !key_names_str.empty());
    
    if (has_single) {
      // 处理逗号分隔的多个键名
      auto keys = splitCommaSeparated(single_key_name);
      keys_to_search.insert(keys_to_search.end(), keys.begin(), keys.end());
    }
    
    if (has_multiple) {
      // 处理 key_names 输入
      auto keys = splitCommaSeparated(key_names_str);
      keys_to_search.insert(keys_to_search.end(), keys.begin(), keys.end());
    }
    
    // 如果两种方式都为空，则返回失败
    if (keys_to_search.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "key_name 和 key_names 均为空或未提供");
      return BT::NodeStatus::FAILURE;
    }
    
    // 去除重复项
    removeDuplicates(keys_to_search);
    
    RCLCPP_DEBUG(node_->get_logger(), "查询 %zu 个键: %s", 
                 keys_to_search.size(), joinStrings(keys_to_search).c_str());
    
    // 输出美化方框
    std::stringstream ss;
    bool found_any = false;
    int found_count = 0;
    
    // 方框顶部
    ss << "\n╔══════════════════════════════════════════════════════════════╗\n";
    ss << "║                   BLACKBOARD 查询结果                      ║\n";
    ss << "╠══════════════════════════════════════════════════════════════╣\n";
    
    // 查询每个键
    for (const auto& key_name : keys_to_search) {
      std::string value_str;
      std::string type_str = "未找到";
      
      // 尝试不同类型
      std::string str_val;
      if (blackboard->get<std::string>(key_name, str_val)) {
        value_str = str_val;
        type_str = "字符串";
        found_any = true;
        found_count++;
      } 
      else if (int int_val; blackboard->get<int>(key_name, int_val)) {
        value_str = std::to_string(int_val);
        type_str = "整数";
        found_any = true;
        found_count++;
      }
      else if (bool bool_val; blackboard->get<bool>(key_name, bool_val)) {
        value_str = bool_val ? "true" : "false";
        type_str = "布尔值";
        found_any = true;
        found_count++;
      }
      else if (double double_val; blackboard->get<double>(key_name, double_val)) {
        std::stringstream dss;
        dss << std::fixed << std::setprecision(3) << double_val;
        value_str = dss.str();
        type_str = "浮点数";
        found_any = true;
        found_count++;
      }
      else {
        value_str = "[NULL]";
      }
      
      // 输出单行结果（限制值长度）
      if (value_str.length() > 25) {
        value_str = value_str.substr(0, 22) + "...";
      }
      
      ss << "║ 键: " << std::left << std::setw(15) << key_name 
         << " │ 类型: " << std::setw(8) << type_str 
         << " │ 值: " << std::setw(20) << value_str << " ║\n";
      
      if (&key_name != &keys_to_search.back()) {
        ss << "╠──────────────────────────────────────────────────────────────╣\n";
      }
    }
    
    // 方框底部
    ss << "╚══════════════════════════════════════════════════════════════╝\n";
    
    // 输出结果
    RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
    
    // 总结信息
    if (found_any) {
      RCLCPP_INFO(node_->get_logger(), "✓ 查询完成: 共 %zu 个键，找到 %d 个", keys_to_search.size(), found_count);
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "✗ 查询失败: 共 %zu 个键，全部未找到", keys_to_search.size());
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  
  // 分割逗号分隔的字符串
  std::vector<std::string> splitCommaSeparated(const std::string& input) {
    std::vector<std::string> result;
    std::stringstream ss(input);
    std::string token;
    
    while (std::getline(ss, token, ',')) {
      // 去除前后空格
      token.erase(0, token.find_first_not_of(" \t\n\r"));
      token.erase(token.find_last_not_of(" \t\n\r") + 1);
      if (!token.empty()) {
        result.push_back(token);
      }
    }
    
    return result;
  }
  
  // 去除重复项
  void removeDuplicates(std::vector<std::string>& vec) {
    std::sort(vec.begin(), vec.end());
    vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
  }
  
  // 将字符串列表连接为逗号分隔的字符串
  std::string joinStrings(const std::vector<std::string>& vec) {
    std::string result;
    for (size_t i = 0; i < vec.size(); ++i) {
      if (i > 0) result += ", ";
      result += vec[i];
    }
    return result;
  }
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<DumpBlackboard>("DumpBlackboard");
}