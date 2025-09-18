std_msgs/Header header
string accid                # 消息顶层的机器人序列号
string title                # 消息类型标题，如 "notify_robot_info"
int64 timestamp             # 消息时间戳，单位为毫秒
string guid                 # 消息的唯一标识符
# 以下字段来自 data 对象
string data_accid           # 数据中的机器人序列号
string sw_version           # 机器人软件版本
string imu_status           # 机器人IMU诊断信息 ("OK" 或错误状态)
string camera_status        # 机器人相机诊断信息 ("OK" 或错误状态)
string motor_status         # 机器人电机诊断信息 ("OK" 或错误状态)
int32 battery               # 机器人电池电量 (百分比, 0-100)
string status               # 机器人运行模式 (STAND, WALK, SIT 等)
string result               # 机器人操作结果