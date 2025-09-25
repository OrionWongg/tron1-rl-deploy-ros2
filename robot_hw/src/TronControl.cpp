#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "robot_hw/RobotControlInterface.h"
#include <unordered_map>
#include <functional>
#include <string>
#include <chrono>
#include <thread>
#include <nav_msgs/msg/odometry.hpp>

class TronCTRLHandlerNode : public rclcpp::Node {
public:
    TronCTRLHandlerNode() : Node("tron_ctrl_node") {
        // 初始化机器人连接
        robot_control_ = std::make_unique<RobotControlInterface>("10.192.1.2", 5000, "WF_TRON1A_204");

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, 
            std::bind(&TronCTRLHandlerNode::cmd_vel_callback, this, std::placeholders::_1));

        // 初始动作序列
        executeInitialActions();
    }

private:
    void executeInitialActions() {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        robot_control_->stand();
        RCLCPP_INFO(this->get_logger(), "机器人已站立");
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "接收到速度命令: linear.x=%f, linear.y=%f, linear.z=%f, angular.z=%f", 
                    msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
        
        // 将接收到的速度命令传递给机器人控制接口
        robot_control_->twist(msg->linear.x, msg->linear.y, msg->angular.z);
        
        // 处理高度调整 (linear.z)
        if (msg->linear.z > 0.1) {
            RCLCPP_INFO(this->get_logger(), "调整高度: 升高");
            robot_control_->adjustHeight(1);
        } else if (msg->linear.z < -0.1) {
            RCLCPP_INFO(this->get_logger(), "调整高度: 降低");
            robot_control_->adjustHeight(-1);
        }
    }

    std::unique_ptr<RobotControlInterface> robot_control_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TronCTRLHandlerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
