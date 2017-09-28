/*
 * main.cpp
 *
 *  Created on: 2017年7月4日
 *      Author: hustac
 */

#include <chrono>
#include <iomanip>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_broadcaster.h>

#include "motor_pid.hpp"
#include "stm32_bridge.hpp"

using namespace LZZ;

static geometry_msgs::Twist twist_msg;

void cmdvelCallback(const geometry_msgs::Twist &msg) {
    twist_msg = msg;
    ROS_DEBUG(
        "[cmd_vel] linear(m/s): %4.2f %4.2f %4.2f angular(deg/s): %d %d %d",
        msg.linear.x, msg.linear.y, msg.linear.z,
        static_cast<int>(msg.angular.x), static_cast<int>(msg.angular.y),
        static_cast<int>(msg.angular.z));
}

static void timer100hzCallback(const ros::TimerEvent &) {
    // publish tf from base_link to kinect_link
//    ptf_broadcaster->sendTransform(tf::StampedTransform(
//        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.1, 0.03, 0.15)),
//        ros::Time::now(), "base_link", "kinect_link"));
    // publish tf from base_link to imu_link
//    ptf_broadcaster->sendTransform(tf::StampedTransform(
//        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.02, 0.02, 0.015)),
//        ros::Time::now(), "base_link", "imu_link"));
}

int main(int argc, char **argv) {
    try {
        // init stm32 bridge
        STM32Bridge bridge(B2000000);
        std::cout << bridge.getSerName() << std::endl;

        // init pid controller
        MotorPID pid[2];

        // init ros node
        ros::init(argc, argv, "stm32_bridge");
        ros::NodeHandle node;

        // init publisher
        ros::Publisher pub_wheel =
            node.advertise<std_msgs::Float64MultiArray>("wheel", 2);
        ros::Publisher pub_wheel_filt =
            node.advertise<std_msgs::Float64MultiArray>("wheel_filt", 2);
        ros::Publisher pub_duty_real =
            node.advertise<std_msgs::Float64MultiArray>("duty_real", 2);
        ros::Publisher pub_duty_pid =
            node.advertise<std_msgs::Float64MultiArray>("duty_pid", 2);
        ros::Publisher pub_imu =
            node.advertise<sensor_msgs::Imu>("imu/data_raw", 2);

        // init subscriber
        ros::Subscriber sub = node.subscribe("cmd_vel", 2, cmdvelCallback);

        // statistic uart error rate
        auto tp = std::chrono::high_resolution_clock::now();
        int count_frame = 0;
        int count_error = 0;

        // robot pos
        double x = 0;
        double y = 0;
        double z = 0;
        double th = 0;

        // use 1 async process thread
        ros::AsyncSpinner spinner(1);
        spinner.start();

        while (ros::ok()) {
            Message msg = bridge.parse();
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
                    std_msgs::Float64MultiArray ros_msg;
                    ros_msg.data.resize(2, 0);
                    ros_msg.data[0] = (reinterpret_cast<float *>(msg.content.data()))[0];
                    ros_msg.data[1] = (reinterpret_cast<float *>(msg.content.data()))[1];
                    pub_duty_real.publish(ros_msg);
                } else if (msg.type == MessageType::MY_MESSAGE_ENCODER) {
                    // encoder msg
                    const Encoder_DataTypedef *dat =
                        reinterpret_cast<Encoder_DataTypedef *>(msg.content.data());
                    bool bad_encoder_data = true;
                    // check encoder data
                    if (dat->period > 0) {
                        std_msgs::Float64MultiArray ros_msg;
                        ros_msg.data.resize(2, 0);
                        ros_msg.data[0] = dat->delta_left / dat->period;
                        ros_msg.data[1] = dat->delta_right / dat->period;
                        if (fabs(ros_msg.data[0]) <= 2 and
                            fabs(ros_msg.data[1]) <= 2) {
                            // encoder data is ok
                            bad_encoder_data = false;

                            // publish wheel speed
                            pub_wheel.publish(ros_msg);

                            // update robot pos from odometer
                            double delta_path = (dat->delta_left + dat->delta_right) / 2;
                            double delta_th = (dat->delta_right - dat->delta_left) / 0.29;

                            double vlen;
                            if (delta_th == .0) {
                                vlen = delta_path;
                            } else {
                                double r = delta_path / delta_th;
                                vlen = sin(delta_th / 2) * r * 2;
                            }
                            double vth = th + (M_PI / 2 - (M_PI - delta_th) / 2);

                            double delta_x = cos(vth) * vlen;
                            double delta_y = sin(vth) * vlen;

                            x += delta_x;
                            y += delta_y;
                            th += delta_th;

                            // broadcast tf from odom to base_link
                            tf_broadcaster.sendTransform(tf::StampedTransform(
                                tf::Transform(tf::createQuaternionFromYaw(th), tf::Vector3(x, y, z)),
                                ros::Time::now(), "odom", "base_footprint"));

                            // pid control
                            std::vector<double> wheel_desired(2, 0);
                            double speed_deriv = 0.291 * twist_msg.angular.z;
                            wheel_desired[0] =
                                twist_msg.linear.x - speed_deriv / 2;
                            wheel_desired[1] =
                                twist_msg.linear.x + speed_deriv / 2;
                            auto tp = std::chrono::high_resolution_clock::now();
                            std_msgs::Float64MultiArray pid_msg;
                            pid_msg.data.resize(2, 0);
                            for (int i = 0; i < 2; i++) {
                                std::string status = pid[i].update(
                                    wheel_desired[i], ros_msg.data[i], tp);
                                if (status.size()) {
                                    ROS_WARN_STREAM("PID[" << i
                                                           << "]: " << status);
                                }
                                pid_msg.data[i] = pid[i].output;
                            }
                            bridge.sendMsgMotor(static_cast<float>(pid_msg.data[0]),
                                                static_cast<float>(pid_msg.data[1]));
                            pub_duty_pid.publish(pid_msg);
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

                    // publish raw gyro&accel msgs
                    sensor_msgs::Imu imu_msg;
                    imu_msg.header.frame_id = "imu_link";
                    imu_msg.header.stamp = ros::Time::now();
                    imu_msg.angular_velocity.x = dat->gyro_x;
                    imu_msg.angular_velocity.y = dat->gyro_y;
                    imu_msg.angular_velocity.z = dat->gyro_z;
                    imu_msg.linear_acceleration.x = dat->accel_x;
                    imu_msg.linear_acceleration.y = dat->accel_y;
                    imu_msg.linear_acceleration.z = dat->accel_z;
                    pub_imu.publish(imu_msg);

                    // publish raw mag field msgs
//                    sensor_msgs::MagneticField mag_msg;
//                    mag_msg.magnetic_field.x = dat->
                }
            }

            // log error/frame
            if (std::chrono::high_resolution_clock::now() - tp >
                std::chrono::seconds(1)) {
                tp = std::chrono::high_resolution_clock::now();
                std::ostringstream oss;
                oss << "Error rate: " << std::setw(6) << std::showpoint
                    << std::setprecision(2) << std::fixed
                    << static_cast<float>(count_error) * 100 / (count_frame + count_error)
                    << "%(" << count_frame << " frames, " << count_error
                    << " errors)";
                ROS_INFO_STREAM(oss.str());
                count_frame = 0;
                count_error = 0;
            }
        }
        bridge.close();
    } catch (const std::system_error &err) {
        std::cerr << "Error " << err.code() << ": " << err.what() << std::endl;
    }
    return 0;
}
