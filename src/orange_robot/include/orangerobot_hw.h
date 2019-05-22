#ifndef ORANGEROBOT_HW_H
#define ORANGEROBOT_HW_H

#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "motor_pid.hpp"
#include "stm32_bridge.hpp"

namespace LZZ {

class OrangeRobot : public hardware_interface::RobotHW {
  public:
    OrangeRobot();

    virtual ~OrangeRobot();

    virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);

    virtual void read();

    virtual void read(const ros::Time &, const ros::Duration &) { read(); }

    virtual void write();

    virtual void write(const ros::Time &, const ros::Duration &) { write(); }

    ros::Time get_time() const { return update_time; }

    ros::Duration get_period() const { return update_period; }

  private:

    double wheel_radius =
        0; // auto read from urdf "wheel_left_joint/collision/geometry/radius"

    ros::Time update_time; // this read() time point
    ros::Duration update_period; // period between this and last read()

    // mpu6050 data
    double gyro[3];  // rad/s
    double accel[3]; // m/s^2
    double zeros[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    hardware_interface::ImuSensorHandle::Data imu_data;

    // wheel data
    double pos[2]; // position, rad
    double vel[2]; // m/s
    double eff[2]; // duty, [-1, 1]

    // control data
    double cmd[2] = {0, 0}; // desired speed, rad/s

    hardware_interface::JointStateInterface jnt_wheel_state;
    hardware_interface::VelocityJointInterface jnt_wheel_vel;
    hardware_interface::ImuSensorInterface jnt_imu;

    ros::Publisher pub_wheel;
    ros::Publisher pub_duty;
    ros::ServiceServer service_start;
    ros::ServiceServer service_stop;

    STM32Bridge *stm32_bridge = nullptr;
    bool is_stm32_online = true;
    // stm32_time + stm32_time_diff should = ros_time
    ros::Duration stm32_time_diff;

    // Motor PID controller
    MotorPID pid[2];
    double old_pos[2] = {0, 0};
    ros::Time old_time;

    // statistic serial error rate
    int count_frame = 0;
    int count_error = 0;

    void recv_msg_forever();
    void proc_msg_forever();

    std::thread thread_recv_msg;
    std::thread thread_proc_msg;

    std::queue<STM32Message> queue_msg;
    std::mutex mutex_msg;
    std::condition_variable cv_msg;

    std::condition_variable cv_data_ready;  // notofied when data ready
    bool data_ready = false;
    std::mutex mutex_buf; // lock the follwing member
    double buf_gyro[3];
    double buf_accel[3];
    double buf_pos[2];
    double buf_vel[2];
    double buf_eff[2];
    double buf_cmd[2];
    ros::Time has_new_imu;      // time when read new IMU data
    ros::Time has_new_encoder;  // time when read new encoder data
};
}

#endif // ORANGEROBOT_HW_H
