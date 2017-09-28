#include "orangerobot_hw.h"

#include <std_msgs/Float64MultiArray.h>

using namespace LZZ;

OrangeRobot::OrangeRobot() {

    // registe left wheel to wheel interface
    hardware_interface::JointStateHandle handle_state_wheel_left(
        "wheel_left", nullptr, &vel[0], nullptr);
    hardware_interface::JointHandle handle_wheel_left(handle_state_wheel_left,
                                                      &eff[0]);
    jnt_wheel.registerHandle(handle_wheel_left);

    // registe right wheel to wheel interface
    hardware_interface::JointStateHandle handle_state_wheel_right(
        "wheel_right", nullptr, &vel[0], nullptr);
    hardware_interface::JointHandle handle_wheel_right(handle_state_wheel_right,
                                                       &eff[0]);
    jnt_wheel.registerHandle(handle_wheel_right);

    // registe wheel interface to robot
    this->registerInterface(&jnt_wheel);

    // registe imu handle to robot
    imu_data.name = "MPU6050";
    imu_data.orientation = zeros;
    imu_data.orientation_covariance = zeros;
    imu_data.angular_velocity = gyro;
    imu_data.angular_velocity_covariance = zeros;
    imu_data.linear_acceleration = accel;
    imu_data.linear_acceleration_covariance = zeros;
    this->registerInterface(&jnt_imu);

    this->stm32_bridge = new STM32Bridge(B2000000);
    ROS_INFO_STREAM(
        "Auto-selected serial port: " << stm32_bridge->getSerName());
}

bool OrangeRobot::init(ros::NodeHandle &, ros::NodeHandle &robot_hw_nh) {
    pub_wheel = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("wheel", 2);
    pub_duty = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("duty", 2);
    return true;
}

void OrangeRobot::thread_recv_msg() {

    while (ros::ok()) {
        // recv a msg
        Message msg = stm32_bridge->parse();

        // lock sensor_data
        mutex_data.lock();
        if (msg.type == MessageType::MY_MESSAGE_NONE) {
            count_error++;
        } else {
            // handler msg
            ROS_DEBUG_STREAM(msg);
            count_frame++;
            if (msg.type == MessageType::MY_MESSAGE_STRING) {
                // string msg
                ROS_INFO_STREAM(reinterpret_cast<char *>(msg.content.data()));
            } else if (msg.type == MessageType::MY_MESSAGE_MOTOR) {
                // motor duty msg
                std_msgs::Float64MultiArray duty_msg;
                duty_msg.data.resize(2, 0);
                duty_msg.data[0] =
                    (reinterpret_cast<float *>(msg.content.data()))[0];
                duty_msg.data[1] =
                    (reinterpret_cast<float *>(msg.content.data()))[1];
                pub_duty.publish(duty_msg);
            } else if (msg.type == MessageType::MY_MESSAGE_ENCODER) {
                // encoder msg
                const Encoder_DataTypedef *dat =
                    reinterpret_cast<Encoder_DataTypedef *>(msg.content.data());
                bool bad_encoder_data = true;
                // check encoder data
                if (dat->period > 0) {
                    std_msgs::Float64MultiArray wheel_msg;
                    wheel_msg.data.resize(2, 0);
                    wheel_msg.data[0] = dat->delta_left / dat->period;
                    wheel_msg.data[1] = dat->delta_right / dat->period;
                    if (fabs(wheel_msg.data[0]) <= 2 and
                        fabs(wheel_msg.data[1]) <= 2) {
                        // encoder data is ok
                        bad_encoder_data = false;

                        vel[0] = wheel_msg.data[0];
                        vel[1] = wheel_msg.data[1];
                        has_new_encoder = true;

                        pub_wheel.publish(wheel_msg);
                    }
                }
                if (bad_encoder_data) {
                    // encoder data incorrect
                    ROS_WARN_STREAM("Encoder data incorrect, " << msg);
                }
            } else if (msg.type == MessageType::MY_MESSAGE_IMU) {
                // encoder msg
                const MPU_DataTypedef *dat =
                    reinterpret_cast<MPU_DataTypedef *>(msg.content.data());

                imu_data.frame_id = "imu_link";
                gyro[0] = dat->gyro_x;
                gyro[1] = dat->gyro_y;
                gyro[2] = dat->gyro_z;
                accel[0] = dat->accel_x;
                accel[0] = dat->accel_y;
                accel[0] = dat->accel_z;
                has_new_imu = true;
            }
        }
        mutex_data.unlock();
    }
}

void OrangeRobot::read() {
    bool recved_timestamp = false;
    bool recved_imu = false;
    bool recved_encoder = false;
    bool recved_motor = false;
    while (!recved_timestamp || !recved_imu || !recved_encoder ||
           !recved_motor) {
        Message msg = stm32_bridge->parse();
        if (msg.type == MessageType::MY_MESSAGE_TIMESTAMP)
    }
}

void OrangeRobot::write() {}
