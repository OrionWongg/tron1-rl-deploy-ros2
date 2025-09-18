#ifndef ROBOT_CONTROL_INTERFACE_H
#define ROBOT_CONTROL_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/robot_command.hpp>
#include <robot_msgs/msg/robot_info.hpp>
#include <robot_msgs/msg/robot_response.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio.hpp>
#include <nlohmann/json.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp> 
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using json = nlohmann::json;
using client = websocketpp::client<websocketpp::config::asio>;
using websocketpp::connection_hdl;

class RobotControlInterface {
public:
    // 添加 use_simulation 参数
    RobotControlInterface(const std::string& robot_ip, int port, const std::string& accid, bool use_simulation = false); //
    ~RobotControlInterface();

    void stand();
    void walk();
    void sitdown();
    void adjustHeight(int direction);
    void emergencyStop();
    void twist(double x, double y, double z);
    void enableOdom(bool enable);
    void enableIMU(bool enable);
    void setStairMode(bool enable); 

private:
    std::string robot_ip;
    int port;
    std::string accid;
    std::string ws_url;
    client ws_client_;
    std::thread ws_thread_;
    std::atomic<bool> is_connected_{false};
    connection_hdl current_hdl_;
    rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr command_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    // notify话题
    rclcpp::Publisher<robot_msgs::msg::RobotInfo>::SharedPtr robot_info_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_; // 新增IMU数据发布器
    rclcpp::Publisher<robot_msgs::msg::RobotResponse>::SharedPtr response_publisher_; // 添加响应消息发布器
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr battery_publisher_;
    
    // 添加消息计数器
    std::atomic<uint64_t> command_count_{0};
    std::atomic<uint64_t> info_count_{0};
    std::atomic<uint64_t> odom_count_{0};
    std::atomic<uint64_t> imu_count_{0};
    std::atomic<uint64_t> response_count_{0};

    void publish_robot_info(const json& message);
    void publish_response(const json& response);
    bool should_publish_notification(const std::string& title);
    bool use_simulation_; // 新增成员，表示是否处于仿真模式

    void connect();
    std::string generate_message(const std::string& title, const json& data = json::object());
    void send_message(const std::string& message);
    void publish_command(const std::string& json_message);
    void handle_message(client::message_ptr msg);
    void publish_odom(const json& odom_data);
    void publish_imu(const json& imu_data);
};

#endif // ROBOT_CONTROL_INTERFACE_H