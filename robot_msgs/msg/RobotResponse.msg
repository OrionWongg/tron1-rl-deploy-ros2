std_msgs/Header header
string accid                # 机器人序列号
string title                # 响应指令名称，以"response_"为前缀
int64 timestamp             # 响应时间戳，单位为毫秒
string guid                 # 与对应请求指令的guid值相同
string result               # 响应结果，值为"success"或以"fail_"为前缀的错误标识