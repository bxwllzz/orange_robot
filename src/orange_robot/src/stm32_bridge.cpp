#include "stm32_bridge.hpp"

#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <dirent.h>
#include <string.h>

#include <iomanip>
#include <sstream>
#include <system_error>
#include <chrono>

namespace LZZ {

Message::Message() :
        type(MessageType::MY_MESSAGE_NONE), crc(0) {
}

void Message::calcCRC() {
    crc = calc_crc8(content.data(), content.size());
}

uint8_t calc_crc8(uint8_t *ptr, size_t len) {
    size_t i;
    uint8_t crc = 0;
    while (len-- != 0) {
        for (i = 0x80; i != 0; i /= 2) {
            if ((crc & 0x80) != 0) {
                crc *= 2;
                crc ^= 7;
            } /* 余式CRC 乘以2 再求CRC */
            else
                crc *= 2;
            if ((*ptr & i) != 0)
                crc ^= 0x7; /* 再加上本位的CRC */
        }
        ptr++;
    }
    return (crc);
}

std::ostream& operator<<(std::ostream & os, Message msg) {
    Encoder_DataTypedef* enc_dat;
    MPU_DataTypedef* mpu_dat;
    float* motor_dat;

    switch (msg.type) {
    case MessageType::MY_MESSAGE_STRING:
        os << "Message type: " << (int) msg.type << "(String), content: "
                << (char*) msg.content.data();
        break;
    case MessageType::MY_MESSAGE_TIMESTAMP:
        os << "Message type: " << (int) msg.type << "(TimeStamp), content: "
                << *(double*) msg.content.data();
        break;
    case MessageType::MY_MESSAGE_ENCODER:
        enc_dat = (Encoder_DataTypedef*) msg.content.data();
        os << "Message type: " << (int) msg.type << "(Encoder)"
                << ", timestamp: " << enc_dat->timestamp << ", period: "
                << enc_dat->period << ", distance_left: "
                << enc_dat->distance_left << ", distance_right: "
                << enc_dat->distance_right << ", delta_left: "
                << enc_dat->delta_left << ", delta_right: "
                << enc_dat->delta_right;
        break;
    case MessageType::MY_MESSAGE_IMU:
        mpu_dat = (MPU_DataTypedef*) msg.content.data();
        os << "Message type: " << (int) msg.type << "(IMU)" << ", timestamp: "
                << mpu_dat->timestamp << ", temprature: " << mpu_dat->temprature
                << ", accel: " << mpu_dat->accel_x << " " << mpu_dat->accel_y
                << " " << mpu_dat->accel_z << ", gyro: " << mpu_dat->gyro_x
                << " " << mpu_dat->gyro_z << " " << mpu_dat->gyro_z;
        ;
        break;
    case MessageType::MY_MESSAGE_MOTOR:
        motor_dat = (float*) msg.content.data();
        os << "Message type: " << (int) msg.type << "(Motor), content: "
                << motor_dat[0] << " " << motor_dat[1];
        break;
    case MessageType::MY_MESSAGE_NONE:
        os << "Message type: " << (int) msg.type << "(None)";
        break;
    default:
        os << "Message type: " << (int) msg.type << ", content length: "
                << msg.content.size();
    }
    return os;
}

// wrap of ::read(), block until nbytes recved or timeout
// if timeout, buffer recved data
// if timeout_n100ms <= 0, block forever
int STM32Bridge::read(uint8_t *buf, size_t nbytes, int timeout_n100ms) {
    size_t recved_nbytes = 0;
    // copy data from buffer
    if (recv_buf.size() > 0) {
        size_t n;
        if (recv_buf.size() >= nbytes) {
            n = nbytes;
        } else {
            n = recv_buf.size();
        }
        memcpy(buf, recv_buf.data(), n);
        recved_nbytes += n;
        // adjust recv_buf
        memmove(recv_buf.data(), recv_buf.data() + n, recv_buf.size() - n);
        recv_buf.resize(recv_buf.size() - n);
    }
    // read from serial
    auto start_time = std::chrono::high_resolution_clock::now();
    while (recved_nbytes < nbytes) {
        if (timeout_n100ms > 0
                && std::chrono::high_resolution_clock::now() - start_time
                        >= std::chrono::milliseconds(100 * timeout_n100ms)) {
            break;
        }
        int ret = ::read(ser_no, buf + recved_nbytes, nbytes - recved_nbytes);
        if (ret < 0) {
            throw std::system_error(errno, std::system_category());
        }
        recved_nbytes += ret;
    }
    if (recved_nbytes == nbytes) {
        return 0;
    } else {
        // timeout, data save into buffer
        size_t len = recv_buf.size();
        recv_buf.resize(len + recved_nbytes);
        memcpy(recv_buf.data() + len, buf, recved_nbytes);
        return -1;
    }
}

STM32Bridge::STM32Bridge(speed_t baurate) {
    // 遍历/dev/ttyUSBx, 查找并打开串口
    DIR *dir;
    ser_no = -2;
    if ((dir = opendir("/dev/")) != NULL) {
        struct dirent *ent;
        while ((ent = readdir(dir)) != NULL) {
            std::string prefix("ttyUSB");
            std::string filename(ent->d_name);
            if (filename.compare(0, prefix.size(), prefix) == 0) {
                ser_name = "/dev/" + filename;
                // 以O_NDELAY打开串口, 避免等待DCD信号线
                ser_no = -1;
                ser_no = open(ser_name.c_str(),
                O_RDWR | O_NOCTTY | O_NDELAY);
                if (ser_no == 0) {
                    break;
                }
            }
        }
        closedir(dir);
        if (ser_no < 0) {
            if (ser_no == -2) {
                throw std::runtime_error("Cannot find /dev/ttyUSB*");
            } else {
                throw std::system_error(errno, std::system_category());
            }
        }
    } else {
        std::cerr << "Cannot open /dev" << std::endl;
        throw std::system_error(errno, std::system_category());
    }
    // 配置为阻塞模式
    int flags = fcntl(ser_no, F_GETFL, 0);
    flags &= ~O_NONBLOCK;
    fcntl(ser_no, F_SETFL, flags);
    // 获取串口配置
    struct termios tty;
    if (tcgetattr(ser_no, &tty) != 0) {
        throw std::system_error(errno, std::system_category());
    }
    // 修改串口配置
    cfsetospeed(&tty, B2000000); // 输出波特率为2M
    cfsetispeed(&tty, B2000000); // 输入波特率为2M
    tty.c_iflag = 0; // 默认终端输入
    tty.c_oflag = 0; // 默认终端输出
    tty.c_cflag &= ~CSIZE; // 数据长度8bit
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CSTOPB; // 单停止位
    tty.c_cflag |= CREAD;
    tty.c_cflag &= ~PARENB; // 无奇偶校验
    tty.c_cflag &= ~PARODD;
    tty.c_cflag &= ~CLOCAL; // 忽略调制解调器线路状态
    tty.c_cflag &= ~CRTSCTS; // 不使用硬件流控
    tty.c_lflag = 0; // 不使用终端编辑功能
    tty.c_cc[VMIN] = 0; // 仅当超时或数据大于nbytes时返回
    tty.c_cc[VTIME] = 5; // 读取超时为0.1s
    // 清除输入输出缓冲
    if (tcflush(ser_no, TCIOFLUSH) != 0) {
        throw std::system_error(errno, std::system_category());
    }
    // 应用串口配置, 并立即生效
    if (tcsetattr(ser_no, TCSANOW, &tty) != 0) {
        throw std::system_error(errno, std::system_category());
    }

    sendFlush();

    recv_buf.reserve(1024);
}

// recv a msg from serial
// or wait for bus re-sync and return a NONE msg
Message STM32Bridge::parse() {
    try {
        Message msg;
        // read message type
        do {
            read((uint8_t*) &msg.type, 1);  // no timeout, wait forever
        } while (msg.type == MessageType::MY_MESSAGE_NONE);
        // read message content
        switch (msg.type) {
        case MessageType::MY_MESSAGE_STRING:
            msg.content.reserve(32);
            while (true) {
                uint8_t c;
                if (read(&c, 1, 1) < 0) {
                    // error, timeout
                    throw std::string("Timeout read content string");
                }
                msg.content.push_back(c);
                if (c == 0) {
                    // \0, string end
                    break;
                }
            }
            break;
        case MessageType::MY_MESSAGE_TIMESTAMP:
            msg.content.resize(sizeof(double));
            if (read(msg.content.data(), msg.content.size(), 1) < 0) {
                // error, timeout
                throw std::string("Timeout read content timestamp");
            }
            break;
        case MessageType::MY_MESSAGE_ENCODER:
            msg.content.resize(sizeof(Encoder_DataTypedef));
            if (read(msg.content.data(), msg.content.size(), 1) < 0) {
                // error, timeout
                throw std::string("Timeout read content encoder");
            }
            break;
        case MessageType::MY_MESSAGE_IMU:
            msg.content.resize(sizeof(MPU_DataTypedef));
            if (read(msg.content.data(), msg.content.size(), 1) < 0) {
                // error, timeout
                throw std::string("Timeout read content IMU");
            }
            break;
        case MessageType::MY_MESSAGE_MOTOR:
            msg.content.resize(sizeof(float) * 2);
            if (read(msg.content.data(), msg.content.size(), 1) < 0) {
                // error, timeout
                throw std::string("Timeout read content motor");
            }
            break;
        default:
            // error, bad message type
            std::ostringstream oss;
            oss << "Bad msg type " << (int) msg.type;
            throw std::string(oss.str());
        }
        // read and check crc8 value
        if (read(&msg.crc, 1, 1) < 0) {
            throw std::string("Timeout read crc8");
        }
        uint8_t crc = calc_crc8(msg.content.data(), msg.content.size());
        if (crc != msg.crc) {
            // error, bad crc8
            std::ostringstream oss;
            oss << "Bad CRC8 value " << (int) msg.crc << "(" << (int) crc
                    << " expected)";
            throw std::string(oss.str());
        }
        return msg;
    } catch (const std::string& err_str) {
        // bus error detected
//            std::cout << "Handle Error(" << err_str << ")" << std::flush;
        // wait for 16x"\0" to sync frame
        uint8_t dat[16];
        bool fullzero = false;
        while (!fullzero) {
            read(dat, 16);  // no timeout, wait forever
            fullzero = true;
            for (int i = 0; i < 16; i++) {
                if (dat[i] != 0) {
                    fullzero = false;
                    break;
                }
            }
//                std::cout << "." << std::flush;
        }
//            std::cout << "OK" << std::endl;
        return Message();
    }
}

// send 16x'\0' to re-sync
int STM32Bridge::sendFlush(int timeout_n100ms) {
    std::vector<uint8_t> dat(32, 0);
    auto start_time = std::chrono::high_resolution_clock::now();
    size_t written = 0;
    while (written < dat.size()) {
        if (timeout_n100ms > 0
                && std::chrono::high_resolution_clock::now() - start_time
                        >= std::chrono::milliseconds(100 * timeout_n100ms)) {
            return -1;
        }
        size_t n = write(ser_no, dat.data() + written, dat.size());
        if (n < 0) {
            throw std::system_error(errno, std::system_category());
        }
        written += n;
    }
    return 0;
}

int STM32Bridge::sendMsg(Message& msg, int timeout_n100ms) {
    msg.calcCRC();
    if (write(ser_no, &msg.type, 1) != 1) {
        throw std::system_error(errno, std::system_category());
    }
    auto start_time = std::chrono::high_resolution_clock::now();
    size_t written = 0;
    while (written < msg.content.size()) {
        if (timeout_n100ms > 0
                && std::chrono::high_resolution_clock::now() - start_time
                        >= std::chrono::milliseconds(100 * timeout_n100ms)) {
            return -1;
        }
        size_t n = write(ser_no, msg.content.data() + written,
                msg.content.size());
        if (n < 0) {
            throw std::system_error(errno, std::system_category());
        }
        written += n;
    }
    if (write(ser_no, &msg.crc, 1) != 1) {
        throw std::system_error(errno, std::system_category());
    }
    return 0;
}

int STM32Bridge::sendMsgMotor(float left, float right) {
    Message msg;
    msg.type = MessageType::MY_MESSAGE_MOTOR;
    msg.content.resize(sizeof(float) * 2);
    float *dat = (float*) msg.content.data();
    dat[0] = left;
    dat[1] = right;
    return sendMsg(msg);
}

int STM32Bridge::sendMsgString(std::string str) {
    Message msg;
    msg.type = MessageType::MY_MESSAGE_STRING;
    msg.content.resize(str.size() + 1);
    memcpy(msg.content.data(), str.c_str(), str.size() + 1);
    return sendMsg(msg);
}

void STM32Bridge::close() {
    sendMsgMotor(0, 0);
    if (::close(ser_no) < 0) {
        throw std::system_error(errno, std::system_category());
    }
    ser_no = -1;
}

STM32Bridge::~STM32Bridge() {
    try {
        sendMsgMotor(0, 0);
        close();
    } catch (const std::system_error& err) {
    }
}

const std::string& STM32Bridge::getSerName() {
    return ser_name;
}

}

