#ifndef ORANGEROBOT_HW_H
#define ORANGEROBOT_HW_H

#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include "stm32_bridge.hpp"
#include "motor_pid.hpp"

namespace LZZ {

class OrangeRobot : public hardware_interface::RobotHW {
public:

    OrangeRobot();

    virtual ~OrangeRobot();

    virtual bool init(ros::NodeHandle & root_nh, ros::NodeHandle &robot_hw_nh);

    virtual void read();

    virtual void read(const ros::Time& , const ros::Duration&) {
        read();
    }

    virtual void write();

    virtual void write(const ros::Time& , const ros::Duration&) {
        write();
    }

    ros::Time get_time() const {
        return update_time;
    }

    ros::Duration get_period() const {
        return update_period;
    }

private:

    bool start_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    bool stop_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    
    double wheel_radius = 0;

    ros::Time update_time;
    ros::Duration update_period{ros::Rate(200)};

    // mpu6050 data
    double gyro[3]; // rad/s
    double accel[3];// m/s^2
    double zeros[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    hardware_interface::ImuSensorHandle::Data imu_data;

    // wheel data
    double pos[2];  // position, rad
    double vel[2];  // m/s
    double eff[2];  // duty, [-1, 1]

    // control data
    double cmd[2] = {0, 0};  // desired speed, rad/s

    hardware_interface::JointStateInterface jnt_wheel_state;
    hardware_interface::VelocityJointInterface jnt_wheel_vel;
    hardware_interface::ImuSensorHandle jnt_imu{imu_data};

    ros::Publisher pub_wheel;
    ros::Publisher pub_duty;
    ros::ServiceServer service_start;
    ros::ServiceServer service_stop;
    
    STM32Bridge *stm32_bridge = nullptr;

    // Motor PID controller
    MotorPID pid[2];
    double old_pos[2] = {0, 0};
    ros::Time old_time;

    // statistic serial error rate
    int count_frame = 0;
    int count_error = 0;

    void recv_msg_forever();
    std::thread thread_recv_msg;
    bool needDestroy = false;       // set this true to exit thread_recv_msg

    std::mutex mutex_buf;           // lock the follwing member
    double buf_gyro[3];
    double buf_accel[3];
    double buf_pos[2];
    double buf_vel[2];
    double buf_eff[2];
    double buf_cmd[2];
    ros::Time update_time_buf;
    bool has_new_imu = false;
    bool has_new_encoder = false;

    std::mutex mutex_has_new;       // unlocked when has new sensor data
};

}

#endif // ORANGEROBOT_HW_H
