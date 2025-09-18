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
        // 声明并获取仿真参数
        this->declare_parameter<bool>("use_simulation", false); //
        bool use_simulation = this->get_parameter("use_simulation").as_bool(); //

        // 初始化机器人连接，并传递仿真标志
        robot_control_ = std::make_unique<RobotControlInterface>("10.192.1.2", 5000, "WF_TRON1A_204", use_simulation); //


        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, 
            std::bind(&TronCTRLHandlerNode::cmd_vel_callback, this, std::placeholders::_1));

        // 创建里程计话题订阅者
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&TronCTRLHandlerNode::odom_callback, this, std::placeholders::_1)); //

        // 初始动作序列
        executeInitialActions(); //

        // 启用里程计数据推送
        robot_control_->enableOdom(true); //
        robot_control_->enableIMU(true); //
        RCLCPP_INFO(this->get_logger(), "已发送启用里程计和IMU的请求"); //
    }

private:

    void executeInitialActions() {
        // 这些动作现在在仿真中只会发布到 ROS 2 话题，在非仿真模式下会尝试 WebSocket 通信。
        std::this_thread::sleep_for(std::chrono::seconds(3)); //
        robot_control_->stand(); //
        RCLCPP_INFO(this->get_logger(), "机器人已站立"); //
        std::this_thread::sleep_for(std::chrono::seconds(5)); //
        // robot_control_->walk(); //
        // RCLCPP_INFO(this->get_logger(), "机器人已进入行走模式"); //
        // std::this_thread::sleep_for(std::chrono::seconds(5)); //
        
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "接收到速度命令: linear.x=%f, linear.y=%f, linear.z=%f, angular.z=%f", 
                    msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
        
        // 将接收到的速度命令传递给机器人控制接口
        robot_control_->twist(msg->linear.x, msg->linear.y, msg->angular.z);
        
        // 处理高度调整 (linear.z)
        if (msg->linear.z > 0.1) {  // 使用阈值避免小数值波动导致不必要的高度调整
            RCLCPP_INFO(this->get_logger(), "调整高度: 升高");
            robot_control_->adjustHeight(1);
        } else if (msg->linear.z < -0.1) {
            RCLCPP_INFO(this->get_logger(), "调整高度: 降低");
            robot_control_->adjustHeight(-1);
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "接收到里程计数据:"); //
        RCLCPP_DEBUG(this->get_logger(), "  位置: x=%f, y=%f, z=%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z); //
        RCLCPP_DEBUG(this->get_logger(), "  姿态: x=%f, y=%f, z=%f, w=%f", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w); //
        RCLCPP_DEBUG(this->get_logger(), "  线速度: x=%f, y=%f, z=%f", msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z); //
        RCLCPP_DEBUG(this->get_logger(), "  角速度: x=%f, y=%f, z=%f", msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z); //
    }

    // std::unordered_map<std::string, std::function<void(RobotControlInterface*)>> action_map_; //
    std::unique_ptr<RobotControlInterface> robot_control_; //
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_sub_; //
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; //
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; //
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); //
    auto node = std::make_shared<TronCTRLHandlerNode>(); //
    rclcpp::spin(node); //
    rclcpp::shutdown(); //
    return 0; //
}
