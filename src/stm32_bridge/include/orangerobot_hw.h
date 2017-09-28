#ifndef ORANGEROBOT_HW_H
#define ORANGEROBOT_HW_H

#include <mutex>

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <stm32_bridge.hpp>

namespace LZZ {

class OrangeRobot : public hardware_interface::RobotHW {
public:

    OrangeRobot();

    virtual bool init(ros::NodeHandle & root_nh, ros::NodeHandle &robot_hw_nh);

    virtual void read();

    virtual void read(const ros::Time& , const ros::Duration&) {
        read();
    }

    virtual void write();

    virtual void write(const ros::Time& , const ros::Duration&) {
        write();
    }

private:

    // mpu6050 data
    double gyro[3];
    double accel[3];
    double zeros[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    hardware_interface::ImuSensorHandle::Data imu_data;
    // wheel speed, m/s
    double vel[2];
    // motor duty, range: 0-1
    double eff[2];

    hardware_interface::VelocityJointInterface jnt_wheel;
    hardware_interface::ImuSensorHandle jnt_imu = {imu_data};

    ros::Publisher pub_wheel;
    ros::Publisher pub_duty;

    STM32Bridge *stm32_bridge = nullptr;

    // statistic serial error rate
    int count_frame = 0;
    int count_error = 0;

    bool has_new_imu = false;
    bool has_new_encoder = false;
    std::mutex mutex_data;
    bool needDestroy = false;
    void thread_recv_msg();
};

}

#endif // ORANGEROBOT_HW_H
