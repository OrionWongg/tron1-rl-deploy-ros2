#include "robot_hw/RobotControlInterface.h"

std::string generate_guid() {
    boost::uuids::random_generator gen;
    return boost::uuids::to_string(gen());
}

// 在构造函数中添加调试信息，查看计数器初始状态
RobotControlInterface::RobotControlInterface(const std::string& robot_ip, int port, const std::string& accid, bool use_simulation)
    : robot_ip(robot_ip), port(port), accid(accid), use_simulation_(use_simulation) {
    
    // 确保计数器初始化为0
    command_count_ = 0;
    info_count_ = 0;
    odom_count_ = 0;
    imu_count_ = 0;
    response_count_ = 0;
    
    RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), 
               "构造函数开始: info_count_=%lu", info_count_.load());
    
    ws_url = "ws://" + robot_ip + ":" + std::to_string(port);

    auto node = rclcpp::Node::make_shared("robot_control_node");
    
    command_publisher_ = node->create_publisher<robot_msgs::msg::RobotCommand>("robot_commands", 10);
    robot_info_publisher_ = node->create_publisher<robot_msgs::msg::RobotInfo>("robot_info", 10);
    odom_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_publisher_ = node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    response_publisher_ = node->create_publisher<robot_msgs::msg::RobotResponse>("robot_responses", 10);
    battery_publisher_ = node->create_publisher<std_msgs::msg::Int32>("battery_level", 10);

    RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), 
               "发布器创建完成: info_count_=%lu", info_count_.load());

    if (!use_simulation_) {
        ws_client_.init_asio();
        ws_client_.set_open_handler([this](connection_hdl hdl) {
            is_connected_ = true;
            current_hdl_ = hdl;
            RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "WebSocket 连接成功");
        });
        ws_client_.set_close_handler([this](connection_hdl hdl) {
            is_connected_ = false;
            RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "WebSocket 连接关闭");
        });
        ws_client_.set_fail_handler([this](connection_hdl hdl) {
            auto con = ws_client_.get_con_from_hdl(hdl);
            if (con) {
                RCLCPP_ERROR(rclcpp::get_logger("RobotControlInterface"), "WebSocket 连接失败: %s", con->get_ec().message().c_str());
            }
        });
        ws_client_.set_message_handler([this](connection_hdl hdl, client::message_ptr msg) {
            handle_message(msg);
        });

        connect();

        ws_thread_ = std::thread([this]() {
            ws_client_.run();
        });
    } else {
        RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "处于仿真模式，不连接 WebSocket 服务器");
    }
    
    RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), 
               "构造函数结束: info_count_=%lu", info_count_.load());
}

RobotControlInterface::~RobotControlInterface() {
    if (!use_simulation_ && is_connected_) { // 仅在非仿真模式且已连接时关闭
        ws_client_.close(current_hdl_, websocketpp::close::status::normal, "Normal closure"); //
    }
    if (ws_thread_.joinable()) { //
        ws_thread_.join(); //
    }
}

void RobotControlInterface::stand() {
    if (!use_simulation_ && !is_connected_) { // 仅在非仿真模式且未连接时发出警告
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "尚未建立 WebSocket 连接，无法发送指令"); //
        return; //
    }
    std::string msg = generate_message("request_stand_mode"); //
    send_message(msg); // send_message 现在在内部处理仿真检查
    publish_command(msg); //
    RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "已发送站立指令"); //
}

void RobotControlInterface::walk() {
    if (!use_simulation_ && !is_connected_) { //
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "尚未建立 WebSocket 连接，无法发送指令"); //
        return; //
    }
    std::string msg = generate_message("request_walk_mode"); //
    send_message(msg); //
    publish_command(msg); //
    RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "已发送行走指令"); //
}

void RobotControlInterface::sitdown() {
    if (!use_simulation_ && !is_connected_) { //
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "尚未建立 WebSocket 连接，无法发送指令"); //
        return; //
    }
    std::string msg = generate_message("request_sitdown"); //
    send_message(msg); //
    publish_command(msg); //
    RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "已发送蹲下指令"); //
}

void RobotControlInterface::adjustHeight(int direction) {
    if (!use_simulation_ && !is_connected_) { //
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "尚未建立 WebSocket 连接，无法发送指令"); //
        return; //
    }
    json data; //
    data["direction"] = direction; //
    std::string msg = generate_message("request_base_height", data); //
    send_message(msg); //
    publish_command(msg); //
    RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "已发送调整身高指令，方向: %d", direction); //
}

void RobotControlInterface::emergencyStop() {
    if (!use_simulation_ && !is_connected_) { //
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "尚未建立 WebSocket 连接，无法发送指令"); //
        return; //
    }
    std::string msg = generate_message("request_emgy_stop"); //
    send_message(msg); //
    publish_command(msg); //
    
    RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "已发送紧急停止指令"); //
}

void RobotControlInterface::twist(double x, double y, double z) {
    if (!use_simulation_ && !is_connected_) { //
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "尚未建立 WebSocket 连接，无法发送指令"); //
        return; //
    }
    json data; //
    data["x"] = x; //
    data["y"] = y; //
    data["z"] = z; //
    std::string msg = generate_message("request_twist", data); //
    send_message(msg); //
    publish_command(msg); //
    
    RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), 
               "已发送速度控制指令- linear.x: %f, linear.y: %f, angular.z: %f", x, y, z); //
}

void RobotControlInterface::enableOdom(bool enable) {
    if (!use_simulation_ && !is_connected_) { //
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "尚未建立 WebSocket 连接，无法发送里程计使能指令"); //
        return; //
    }
    json data; //
    data["enable"] = enable; //
    std::string msg = generate_message("request_enable_odom", data); //
    send_message(msg); //
    publish_command(msg); //
    RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "已发送里程计使能指令: %s", enable ? "开启" : "禁用"); //
}

void RobotControlInterface::enableIMU(bool enable) {
    if (!use_simulation_ && !is_connected_) { //
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "尚未建立 WebSocket 连接，无法发送IMU使能指令"); //
        return; //
    }
    json data; //
    data["enable"] = enable; //
    std::string msg = generate_message("request_enable_imu", data); //
    send_message(msg); //
    publish_command(msg); //
    RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "已发送IMU使能指令: %s", enable ? "开启" : "禁用"); //
}

void RobotControlInterface::setStairMode(bool enable) {
    if (!is_connected_) {
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "未连接到机器人，楼梯模式请求被忽略");
        return;
    }

    json data;
    data["enable"] = enable;
    
    std::string message = generate_message("request_stair_mode", data);
    
    if (use_simulation_) {
        // 仿真模式下，发布到ROS话题
        publish_command(message);
        RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "仿真模式：%s楼梯模式", 
                   enable ? "开启" : "关闭");
    } else {
        // 实际机器人模式下，通过WebSocket发送
        send_message(message);
        RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "已请求%s楼梯模式", 
                   enable ? "开启" : "关闭");
    }
}

void RobotControlInterface::connect() {
    if (use_simulation_) return; // 在仿真模式下不连接
    websocketpp::lib::error_code ec; //
    client::connection_ptr con = ws_client_.get_connection(ws_url, ec); //
    if (ec) { //
        RCLCPP_ERROR(rclcpp::get_logger("RobotControlInterface"), "连接 WebSocket 服务器时出错: %s", ec.message().c_str()); //
        return; //
    }
    ws_client_.connect(con); //
}

std::string RobotControlInterface::generate_message(const std::string& title, const json& data) {
    json message; //
    message["accid"] = accid; //
    message["title"] = title; //
    message["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>( //
        std::chrono::system_clock::now().time_since_epoch()).count(); //
    message["guid"] = generate_guid(); //
    message["data"] = data; //
    return message.dump(); //
}

void RobotControlInterface::send_message(const std::string& message) {
    if (use_simulation_) { // 在仿真模式下不发送
        RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "处于仿真模式，不发送 WebSocket 消息: %s", message.c_str()); //
        return; //
    }
    try {
        if (is_connected_) { //
            ws_client_.send(current_hdl_, message, websocketpp::frame::opcode::text); //
        } else {
            RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "WebSocket 未连接，无法发送消息"); //
        }
    } catch (const websocketpp::exception& e) { //
        RCLCPP_ERROR(rclcpp::get_logger("RobotControlInterface"), "发送消息时出错: %s", e.what()); //
    }
}

void RobotControlInterface::publish_command(const std::string& json_message) {
    try {
        json message = json::parse(json_message); //
        robot_msgs::msg::RobotCommand cmd; //
        
        // 添加 header
        cmd.header.stamp = rclcpp::Clock().now();
        // 使用计数器作为frame_id
        cmd.header.frame_id = std::to_string(++command_count_);
        
        cmd.accid = message["accid"]; //
        cmd.title = message["title"]; //
        cmd.timestamp = message["timestamp"]; //
        cmd.guid = message["guid"]; //
        
        cmd.x = 0.0; //
        cmd.y = 0.0; //
        cmd.z = 0.0; //
        
        if (message["data"].contains("x")) { //
            cmd.x = message["data"]["x"]; //
        }
        if (message["data"].contains("y")) { //
            cmd.y = message["data"]["y"]; //
        }
        if (message["data"].contains("z")) { //
            cmd.z = message["data"]["z"]; //
        }
        if (message.contains("result")) { //
            cmd.result = message["result"]; //
        }
        
        command_publisher_->publish(cmd); //
        
        RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), 
                   "发布指令话题: accid=%s, title=%s, timestamp=%ld, guid=%s, count=%s",
                   cmd.accid.c_str(), cmd.title.c_str(), cmd.timestamp, cmd.guid.c_str(), cmd.header.frame_id.c_str()); //

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("RobotControlInterface"), "解析消息时出错: %s", e.what());
    }
}

bool RobotControlInterface::should_publish_notification(const std::string& title) {
    return title.substr(0, 7) == "notify_" && 
           title != "notify_odom" && 
           title != "notify_imu";
}


void RobotControlInterface::handle_message(client::message_ptr msg) {
    if (use_simulation_) return; // 在仿真模式下不处理真实消息
    try {
        std::string payload = msg->get_payload();
        json response = json::parse(payload);
        
        if (response.contains("title")) {
            std::string title = response["title"];
            
            if (title == "notify_odom" && response.contains("data")) {
                // 确保将时间戳信息添加到odom_data中
                json odom_data = response["data"];
                if (response.contains("timestamp")) {
                    odom_data["timestamp"] = response["timestamp"];
                }
                publish_odom(odom_data);
            } else if (title == "notify_imu" && response.contains("data")) {
                // 确保将时间戳信息添加到imu_data中
                json imu_data = response["data"];
                if (response.contains("timestamp")) {
                    imu_data["timestamp"] = response["timestamp"];
                }
                publish_imu(imu_data);
            } else if (title.substr(0, 9) == "response_") {
                // 处理响应消息
                publish_response(response);
            } else if (should_publish_notification(title)) {
                // 检查是否是需要发布的其他notify消息（排除odom和imu）
                publish_robot_info(response);
            } else if (response.contains("data") && response["data"].contains("result")) {
                std::string result = response["data"]["result"];
                RCLCPP_INFO(rclcpp::get_logger("RobotControlInterface"), "操作结果: %s", result.c_str());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("RobotControlInterface"), "处理消息时出错: %s", e.what());
    }
}

void RobotControlInterface::publish_robot_info(const json& message) {
    try {

        uint64_t current_count = info_count_.load();
        RCLCPP_DEBUG(rclcpp::get_logger("RobotControlInterface"), 
                   "准备发布robot_info: 当前count=%lu, 递增后将为=%lu", 
                   current_count, current_count + 1);
        
        robot_msgs::msg::RobotInfo info_msg;
        
        // 获取机器人时间戳
        int64_t robot_timestamp = 0;
        if (message.contains("timestamp") && message["timestamp"].is_number()) {
            robot_timestamp = message["timestamp"].get<int64_t>();
        }
        
        // 将毫秒时间戳转换为ROS时间
        rclcpp::Time ros_time;
        if (robot_timestamp > 0) {
            // 将毫秒转换为秒和纳秒
            int32_t seconds = static_cast<int32_t>(robot_timestamp / 1000);
            uint32_t nanoseconds = static_cast<uint32_t>((robot_timestamp % 1000) * 1000000);
            ros_time = rclcpp::Time(seconds, nanoseconds);
        } else {
            // 如果没有有效的时间戳，使用当前时间
            ros_time = rclcpp::Clock().now();
            RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "机器人信息中没有有效时间戳，使用本地时间");
        }
        
        // 设置header
        info_msg.header.stamp = ros_time;
        info_msg.header.frame_id = std::to_string(++info_count_);

        // 记录实际使用的frame_id
        RCLCPP_DEBUG(rclcpp::get_logger("RobotControlInterface"), 
                   "实际发布robot_info: frame_id=%s, title=%s", 
                   info_msg.header.frame_id.c_str(), 
                   message.contains("title") ? message["title"].get<std::string>().c_str() : "unknown");
        
        
        // 填充顶层字段
        if (message.contains("accid")) {
            info_msg.accid = message["accid"];
        }
        
        if (message.contains("title")) {
            info_msg.title = message["title"];
        }
        
        if (message.contains("timestamp")) {
            info_msg.timestamp = message["timestamp"];
        }
        
        if (message.contains("guid")) {
            info_msg.guid = message["guid"];
        }
        
        // 初始化默认值
        info_msg.data_accid = "";
        info_msg.sw_version = "";
        info_msg.imu_status = "";
        info_msg.camera_status = "";
        info_msg.motor_status = "";
        info_msg.battery = 0;
        info_msg.status = "";
        info_msg.result = "";

        // 如果是notify_robot_info消息，填充data字段
        if (message.contains("title") && message["title"] == "notify_robot_info" && message.contains("data")) {
            auto& data = message["data"];
            
            if (data.contains("accid")) {
                info_msg.data_accid = data["accid"];
            }
            
            if (data.contains("sw_version")) {
                info_msg.sw_version = data["sw_version"];
            }
            
            if (data.contains("imu")) {
                info_msg.imu_status = data["imu"];
            }
            
            if (data.contains("camera")) {
                info_msg.camera_status = data["camera"];
            }
            
            if (data.contains("motor")) {
                info_msg.motor_status = data["motor"];
            }
            
            if (data.contains("battery")) {
                if (data["battery"].is_number()) {
                    info_msg.battery = data["battery"].get<int32_t>();
                } else if (data["battery"].is_string()) {
                    // 如果是字符串，尝试转换为整数
                    try {
                        info_msg.battery = std::stoi(data["battery"].get<std::string>());
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), 
                                  "无法将电池电量字符串转换为整数: %s", e.what());
                        info_msg.battery = 0;
                    }
                }
            }
            
            if (data.contains("status")) {
                info_msg.status = data["status"];
            }
        }
        // 对于其他类型的notify消息
        else if (message.contains("data") && message["data"].contains("result")) {
            info_msg.result = message["data"]["result"];
        }
        
        // 发布消息
        robot_info_publisher_->publish(info_msg);

        // 单独发布电池电量
        if (info_msg.battery > 0) {  // 只有当电池电量有效时才发布
            std_msgs::msg::Int32 battery_msg;
            battery_msg.data = info_msg.battery;
            battery_publisher_->publish(battery_msg);
            
            RCLCPP_DEBUG(rclcpp::get_logger("RobotControlInterface"), 
                       "发布电池电量: %d%%", battery_msg.data);
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("RobotControlInterface"), 
                   "发布机器人信息: title=%s, status=%s, battery=%d, count=%s",
                   info_msg.title.c_str(), info_msg.status.c_str(), info_msg.battery, info_msg.header.frame_id.c_str());
                   
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("RobotControlInterface"), 
                    "发布机器人信息时出错: %s", e.what());
    }
}

void RobotControlInterface::publish_response(const json& response) {
    try {
        robot_msgs::msg::RobotResponse response_msg;

        // 获取机器人时间戳
        int64_t robot_timestamp = 0;
        if (response.contains("timestamp") && response["timestamp"].is_number()) {
            robot_timestamp = response["timestamp"].get<int64_t>();
        }
        
        // 将毫秒时间戳转换为ROS时间
        rclcpp::Time ros_time;
        if (robot_timestamp > 0) {
            // 将毫秒转换为秒和纳秒
            int32_t seconds = static_cast<int32_t>(robot_timestamp / 1000);
            uint32_t nanoseconds = static_cast<uint32_t>((robot_timestamp % 1000) * 1000000);
            ros_time = rclcpp::Time(seconds, nanoseconds);
        } else {
            // 如果没有有效的时间戳，使用当前时间
            ros_time = rclcpp::Clock().now();
            RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "响应消息中没有有效时间戳，使用本地时间");
        }

        response_msg.header.stamp = ros_time;
        response_msg.header.frame_id = std::to_string(++response_count_);

        if (response.contains("accid")) {
            response_msg.accid = response["accid"];
        }

        if (response.contains("title")) {
            response_msg.title = response["title"];
        }

        if (response.contains("timestamp")) {
            response_msg.timestamp = response["timestamp"];
        }

        if (response.contains("guid")) {
            response_msg.guid = response["guid"];
        }

        // 从data中提取result
        if (response.contains("data") && response["data"].contains("result")) {
            response_msg.result = response["data"]["result"];
        } else if (response.contains("result")) {
            response_msg.result = response["result"];
        } else {
            response_msg.result = "unknown";
        }

        response_publisher_->publish(response_msg);
        
        RCLCPP_DEBUG(rclcpp::get_logger("RobotControlInterface"), 
                   "发布响应消息: title=%s, result=%s, count=%s",
                   response_msg.title.c_str(), response_msg.result.c_str(), response_msg.header.frame_id.c_str());
                   
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("RobotControlInterface"), 
                    "发布响应消息时出错: %s", e.what());
    }
}

void RobotControlInterface::publish_odom(const json& odom_data) {
    nav_msgs::msg::Odometry odom_msg;

    // 获取包含odom_data的消息的时间戳
    // 假设odom_data来自某个完整消息的data字段，需要知道原始消息的时间戳
    // 如果odom_data已经是完整消息，那么可以直接获取时间戳
    int64_t robot_timestamp = 0;
    if (odom_data.contains("timestamp") && odom_data["timestamp"].is_number()) {
        robot_timestamp = odom_data["timestamp"].get<int64_t>();
    }
    
    // 将毫秒时间戳转换为ROS时间
    rclcpp::Time ros_time;
    if (robot_timestamp > 0) {
        // 将毫秒转换为秒和纳秒
        int32_t seconds = static_cast<int32_t>(robot_timestamp / 1000);
        uint32_t nanoseconds = static_cast<uint32_t>((robot_timestamp % 1000) * 1000000);
        ros_time = rclcpp::Time(seconds, nanoseconds);
    } else {
        // 如果没有有效的时间戳，使用当前时间
        ros_time = rclcpp::Clock().now();
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "里程计数据中没有有效时间戳，使用本地时间");
    }

    odom_msg.header.stamp = ros_time;
    odom_msg.header.frame_id = "odom_" + std::to_string(++odom_count_);
    odom_msg.child_frame_id = "base_link";

    // Pose
    if (odom_data.contains("pose_position") && odom_data["pose_position"].is_array() && odom_data["pose_position"].size() == 3) {
        odom_msg.pose.pose.position.x = odom_data["pose_position"][0].get<double>();
        odom_msg.pose.pose.position.y = odom_data["pose_position"][1].get<double>();
        odom_msg.pose.pose.position.z = odom_data["pose_position"][2].get<double>();
    } else {
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "里程计数据中'pose_position'字段无效或缺失");
    }

    if (odom_data.contains("pose_orientation") && odom_data["pose_orientation"].is_array() && odom_data["pose_orientation"].size() == 4) {
        odom_msg.pose.pose.orientation.x = odom_data["pose_orientation"][0].get<double>();
        odom_msg.pose.pose.orientation.y = odom_data["pose_orientation"][1].get<double>();
        odom_msg.pose.pose.orientation.z = odom_data["pose_orientation"][2].get<double>();
        odom_msg.pose.pose.orientation.w = odom_data["pose_orientation"][3].get<double>();
    } else {
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "里程计数据中'pose_orientation'字段无效或缺失");
    }

    // Twist
    if (odom_data.contains("twist_linear") && odom_data["twist_linear"].is_array() && odom_data["twist_linear"].size() == 3) {
        odom_msg.twist.twist.linear.x = odom_data["twist_linear"][0].get<double>();
        odom_msg.twist.twist.linear.y = odom_data["twist_linear"][1].get<double>();
        odom_msg.twist.twist.linear.z = odom_data["twist_linear"][2].get<double>();
    } else {
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "里程计数据中'twist_linear'字段无效或缺失");
    }

    if (odom_data.contains("twist_angular") && odom_data["twist_angular"].is_array() && odom_data["twist_angular"].size() == 3) {
        odom_msg.twist.twist.angular.x = odom_data["twist_angular"][0].get<double>();
        odom_msg.twist.twist.angular.y = odom_data["twist_angular"][1].get<double>();
        odom_msg.twist.twist.angular.z = odom_data["twist_angular"][2].get<double>();
    } else {
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "里程计数据中'twist_angular'字段无效或缺失");
    }

    odom_publisher_->publish(odom_msg);
    RCLCPP_DEBUG(rclcpp::get_logger("RobotControlInterface"), "已发布里程计数据, count=%s", odom_msg.header.frame_id.c_str());
}

void RobotControlInterface::publish_imu(const json& imu_data) {
    sensor_msgs::msg::Imu imu_msg;
    
    // 获取包含imu_data的消息的时间戳
    int64_t robot_timestamp = 0;
    if (imu_data.contains("timestamp") && imu_data["timestamp"].is_number()) {
        robot_timestamp = imu_data["timestamp"].get<int64_t>();
    }
    
    // 将毫秒时间戳转换为ROS时间
    rclcpp::Time ros_time;
    if (robot_timestamp > 0) {
        // 将毫秒转换为秒和纳秒
        int32_t seconds = static_cast<int32_t>(robot_timestamp / 1000);
        uint32_t nanoseconds = static_cast<uint32_t>((robot_timestamp % 1000) * 1000000);
        ros_time = rclcpp::Time(seconds, nanoseconds);
    } else {
        // 如果没有有效的时间戳，使用当前时间
        ros_time = rclcpp::Clock().now();
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "IMU数据中没有有效时间戳，使用本地时间");
    }
    
    // 设置时间戳和坐标系
    imu_msg.header.stamp = ros_time;
    imu_msg.header.frame_id = "imu_" + std::to_string(++imu_count_);
    
    // 设置四元数方向
    if (imu_data.contains("quat") && imu_data["quat"].is_array() && imu_data["quat"].size() == 4) {
        // 注意：数据格式为 [w, x, y, z]，但ROS标准格式为 [x, y, z, w]
        imu_msg.orientation.w = imu_data["quat"][0].get<double>();
        imu_msg.orientation.x = imu_data["quat"][1].get<double>();
        imu_msg.orientation.y = imu_data["quat"][2].get<double>();
        imu_msg.orientation.z = imu_data["quat"][3].get<double>();
    } else {
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "IMU数据中'quat'字段无效或缺失");
        // 设置默认值
        imu_msg.orientation.w = 1.0;
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
    }
    
    // 方向协方差，如果不确定，设置为-1表示未知
    for (size_t i = 0; i < 9; ++i) {
        imu_msg.orientation_covariance[i] = -1;
    }
    
    // 设置角速度
    if (imu_data.contains("gyro") && imu_data["gyro"].is_array() && imu_data["gyro"].size() == 3) {
        imu_msg.angular_velocity.x = imu_data["gyro"][0].get<double>();
        imu_msg.angular_velocity.y = imu_data["gyro"][1].get<double>();
        imu_msg.angular_velocity.z = imu_data["gyro"][2].get<double>();
    } else {
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "IMU数据中'gyro'字段无效或缺失");
        // 设置默认值
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.0;
    }
    
    // 角速度协方差，如果不确定，设置为-1表示未知
    for (size_t i = 0; i < 9; ++i) {
        imu_msg.angular_velocity_covariance[i] = -1;
    }
    
    // 设置线性加速度
    if (imu_data.contains("acc") && imu_data["acc"].is_array() && imu_data["acc"].size() == 3) {
        imu_msg.linear_acceleration.x = imu_data["acc"][0].get<double>();
        imu_msg.linear_acceleration.y = imu_data["acc"][1].get<double>();
        imu_msg.linear_acceleration.z = imu_data["acc"][2].get<double>();
    } else {
        RCLCPP_WARN(rclcpp::get_logger("RobotControlInterface"), "IMU数据中'acc'字段无效或缺失");
        // 设置默认值
        imu_msg.linear_acceleration.x = 0.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 0.0;
    }
    
    // 线性加速度协方差，如果不确定，设置为-1表示未知
    for (size_t i = 0; i < 9; ++i) {
        imu_msg.linear_acceleration_covariance[i] = -1;
    }
    
    // 发布IMU消息
    imu_publisher_->publish(imu_msg);
    RCLCPP_DEBUG(rclcpp::get_logger("RobotControlInterface"), "已发布IMU数据, count=%s", imu_msg.header.frame_id.c_str());
}