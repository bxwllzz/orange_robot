#ifndef STM32_BRIDGE_H
#define STM32_BRIDGE_H

#include <termios.h>    // POSIX terminal control definitions
#include <iostream>
#include <vector>
#include <chrono>

namespace LZZ {

struct MPU_DataTypedef {
    double gyro_x;
    double gyro_y;
    double gyro_z;
    double accel_x;
    double accel_y;
    double accel_z;
    double temprature;
    double timestamp;
};

struct Encoder_DataTypedef {
    float delta_left;       // 左轮本次测量的行驶路程
    float delta_right;      // 右轮本次测量的行驶路程
    double distance_left;   // 左轮累计行驶距离
    double distance_right;  // 右轮累计行驶距离
    float period;           // 本次测量周期
    double timestamp;       // 本次测量结束时间
};

enum class STM32MessageType {
    // 无数据
    MY_MESSAGE_NONE = 0,
    // C语言字符串, 可变长度
    MY_MESSAGE_STRING = 1,      // 文本消息
    // 二进制, 固定长度
    MY_MESSAGE_TIMESTAMP = 2,   // 时间戳
    MY_MESSAGE_ENCODER = 3,     // 编码器数据
    MY_MESSAGE_IMU = 4,         // 惯性传感器数据
    MY_MESSAGE_MOTOR = 5,       // 设定电机PWM占空比指令, float float, 0~1
};

class STM32Message {
public:
    STM32MessageType type = STM32MessageType::MY_MESSAGE_NONE;
    std::vector<uint8_t> content;
    std::chrono::high_resolution_clock::time_point tp;
    uint8_t crc = 0;
    void calcCRC();
};

uint8_t calc_crc8(uint8_t *ptr, size_t len);

std::ostream& operator<<(std::ostream & os, STM32Message msg);

class STM32Bridge {
private:
    int ser_no;
    std::string ser_name;
    std::vector<uint8_t> recv_buf;

    // wrap of ::read(), block until nbytes recved or timeout
    // if timeout, buffer recved data
    // if timeout_n100ms <= 0, block forever
    int read(uint8_t *buf, size_t nbytes, int timeout_n100ms = 0);
public:
    STM32Bridge(speed_t baurate);

    ~STM32Bridge();

    const std::string& getSerName();

    // recv a msg from serial
    // or wait for bus re-sync and return a NONE msg
    STM32Message parse();

    int sendFlush(int timeout_n100ms = 0);

    int sendMsg(STM32Message& msg, int timeout_n100ms = 0);

    int sendMsgMotor(float left, float right);

    int sendMsgString(std::string str);

    void close();
};
}
#endif
