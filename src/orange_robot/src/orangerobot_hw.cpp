#include <std_msgs/Float64MultiArray.h>
#include <urdf_parser/urdf_parser.h>

#include "orangerobot_hw.h"

using namespace LZZ;

OrangeRobot::OrangeRobot() {

    // registe left wheel to wheel interface
    hardware_interface::JointStateHandle handle_state_wheel_left(
        "wheel_left_joint", zeros, &vel[0], zeros);
    hardware_interface::JointHandle handle_wheel_left(handle_state_wheel_left,
                                                      &eff[0]);
    jnt_wheel.registerHandle(handle_wheel_left);

    // registe right wheel to wheel interface
    hardware_interface::JointStateHandle handle_state_wheel_right(
        "wheel_right_joint", zeros, &vel[1], zeros);
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
}

bool OrangeRobot::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {

    // registe publishers
    pub_wheel = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("wheel", 2);
    pub_duty = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("duty", 2);

    // start bridge to hardware
    this->stm32_bridge = new STM32Bridge(B2000000);
    ROS_INFO_STREAM(
        "Auto-selected serial port: " << stm32_bridge->getSerName());

    // start serial msg recv thread
    thread_recv_msg = std::thread(&OrangeRobot::recv_msg_forever, this);

    // read wheel radius from URDF
    const std::string model_param_name = "robot_description";
    std::string robot_model_str = "";
    if (!root_nh.hasParam(model_param_name) ||
        !root_nh.getParam(model_param_name, robot_model_str)) {
        ROS_ERROR_NAMED(
            "orange",
            "Robot descripion couldn't be retrieved from param server.");
        return false;
    }
    // get model
    urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robot_model_str));
    // get wheel joint
    urdf::JointConstSharedPtr left_wheel_joint(
        model->getJoint("wheel_left_joint"));
    // get wheel link
    urdf::LinkConstSharedPtr left_wheel_link =
        model->getLink(left_wheel_joint->child_link_name);
    // get cylinder radius
    wheel_radius = (static_cast<urdf::Cylinder *>(
                        left_wheel_link->collision->geometry.get()))
                       ->radius;

    return true;
}

OrangeRobot::~OrangeRobot() {
    // notify thread_recv_msg to exit
    needDestroy = true;
    // wait for thread_recv_msg
    thread_recv_msg.join();

    delete stm32_bridge;
}

void OrangeRobot::recv_msg_forever() {

    while (!needDestroy) {
        // recv a msg
        Message msg = stm32_bridge->parse();

        // lock sensor_data
        mutex_buf.lock();
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
                buf_eff[0] = duty_msg.data[0];
                buf_eff[1] = duty_msg.data[1];
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

                        buf_pos[0] = dat->distance_left / wheel_radius;
                        buf_pos[1] = dat->distance_right / wheel_radius;
                        ros::Time new_time = ros::Time::now();
                        double dt;

                        for (size_t i = 0; i < 2; i++) {
                            // calc wheel speed m/s
                            if (old_time.isZero()) {
                                // without old pos
                                buf_vel[i] = wheel_msg.data[i];
                                dt = static_cast<double>(dat->period);
                            } else {
                                // has old pos
                                buf_vel[i] = (buf_pos[i] - old_pos[i]) *
                                             wheel_radius / dt;
                                dt = (new_time - old_time).toSec();
                            }
                            old_pos[i] = buf_pos[i];
                            old_time = new_time;

                            // PID control
                            std::string status =
                                pid[i].update(buf_cmd[i], buf_vel[i], dt);
                            if (status.size()) {
                                ROS_WARN_STREAM_NAMED("Motor PID", status);
                            }
                        }
                        // send motor duty cmd
                        stm32_bridge->sendMsgMotor(
                            static_cast<float>(pid[0].output),
                            static_cast<float>(pid[1].output));

                        has_new_encoder = true;
                        update_time_buf = ros::Time::now();

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
                buf_gyro[0] = dat->gyro_x;
                buf_gyro[1] = dat->gyro_y;
                buf_gyro[2] = dat->gyro_z;
                buf_accel[0] = dat->accel_x;
                buf_accel[0] = dat->accel_y;
                buf_accel[0] = dat->accel_z;
                has_new_imu = true;
                update_time_buf = ros::Time::now();
            }
        }
        if (has_new_encoder && has_new_imu) {
            mutex_has_new.unlock();
        }
        mutex_buf.unlock();
    }
}

/* wait and read sensor data from hardware */
void OrangeRobot::read() {
    // wait for new sensor data
    mutex_has_new.lock();

    // lock buf
    mutex_buf.lock();

    std::copy(std::begin(buf_gyro), std::end(buf_gyro), std::begin(gyro));
    std::copy(std::begin(buf_accel), std::end(buf_accel), std::begin(accel));
    std::copy(std::begin(buf_vel), std::end(buf_vel), std::begin(vel));
    std::copy(std::begin(buf_pos), std::end(buf_pos), std::begin(pos));
    std::copy(std::begin(buf_eff), std::end(buf_eff), std::begin(eff));
    has_new_encoder = false;
    has_new_imu = false;
    if (!update_time.isZero()) {
        update_period = ros::Duration(update_time_buf - update_time);
    }
    update_time = update_time_buf;
    mutex_has_new.try_lock();

    mutex_buf.unlock();
}

/* wrtie control data to hardware */
void OrangeRobot::write() {
    // lock buf
    std::lock_guard<std::mutex> lock_buf(mutex_buf);

    std::copy(std::begin(cmd), std::end(cmd), std::begin(buf_cmd));
}
