#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include "icp_localization_ros2/msg/icp_quality.hpp"
#include <chrono>
#include <memory>
#include <cmath>

class ICPQualityPublisher : public rclcpp::Node
{
public:
    ICPQualityPublisher() : Node("icp_quality_publisher"), count_(0)
    {
        // 创建发布者
        publisher_ = this->create_publisher<icp_localization_ros2::msg::ICPQuality>(
            "icp_quality", 10);
        
        // 创建定时器，每0.5秒发布一次消息
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ICPQualityPublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "ICP Quality Publisher 已启动");
    }

private:
    void timer_callback()
    {
        auto msg = icp_localization_ros2::msg::ICPQuality();
        
        // 填充 Header
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        
        // 填充模拟的 ICP 质量数据
        msg.convergence_error = 0.01 + 0.005 * sin(count_ * 0.1);  // 正弦波模拟变化
        msg.match_overlap_ratio = 0.85 + 0.1 * sin(count_ * 0.05);
        msg.matched_points_count = 1000 + static_cast<int>(100 * sin(count_ * 0.1));
        msg.fitness_score = 0.92 + 0.06 * sin(count_ * 0.08);
        msg.translation_error = 0.02 + 0.01 * sin(count_ * 0.12);
        msg.rotation_error = 0.005 + 0.003 * sin(count_ * 0.15);
        msg.confidence_score = 0.9 + 0.08 * sin(count_ * 0.07);
        msg.is_converged = (count_ % 20) > 2;  // 模拟收敛状态变化
        msg.iteration_count = 10 + (count_ % 15);
        
        // 发布消息
        publisher_->publish(msg);
        
        // 打印日志
        RCLCPP_INFO(this->get_logger(), "发布第 %zu 条 ICP 质量消息:", count_ + 1);
        RCLCPP_INFO(this->get_logger(), "  收敛误差: %.4f", msg.convergence_error);
        RCPP_INFO(this->get_logger(), "  匹配重叠比例: %.2f", msg.match_overlap_ratio);
        RCLCPP_INFO(this->get_logger(), "  匹配点数量: %d", msg.matched_points_count);
        RCLCPP_INFO(this->get_logger(), "  置信度: %.2f", msg.confidence_score);
        RCLCPP_INFO(this->get_logger(), "  收敛状态: %s", msg.is_converged ? "是" : "否");
        
        count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<icp_localization_ros2::msg::ICPQuality>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ICPQualityPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}