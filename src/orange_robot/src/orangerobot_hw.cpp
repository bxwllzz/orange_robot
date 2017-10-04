#include <std_msgs/Float64MultiArray.h>
#include <urdf_parser/urdf_parser.h>

#include "orangerobot_hw.h"

using namespace LZZ;

static inline ros::Time
chrono_to_ros(const std::chrono::high_resolution_clock::time_point &tp) {
    ros::Time ros_time = ros::Time::now();
    std::chrono::high_resolution_clock::time_point chrono_time =
        std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::duration chrono_dur = chrono_time - tp;
    double sec_dur =
        std::chrono::duration_cast<std::chrono::duration<double>>(chrono_dur)
            .count();
    ros::Duration ros_dur = ros::Duration(sec_dur);
    return ros_time - ros_dur;
}

OrangeRobot::OrangeRobot() {

    // registe left wheel to wheel interface
    hardware_interface::JointStateHandle handle_state_wheel_left(
        "wheel_left_joint", &pos[0], &vel[0], &eff[0]);
    jnt_wheel_state.registerHandle(handle_state_wheel_left);

    hardware_interface::JointHandle handle_wheel_left(
        jnt_wheel_state.getHandle("wheel_left_joint"), &cmd[0]);
    jnt_wheel_vel.registerHandle(handle_wheel_left);

    // registe right wheel to wheel interface
    hardware_interface::JointStateHandle handle_state_wheel_right(
        "wheel_right_joint", &pos[1], &vel[1], &eff[1]);
    jnt_wheel_state.registerHandle(handle_state_wheel_right);

    hardware_interface::JointHandle handle_wheel_right(
        jnt_wheel_state.getHandle("wheel_right_joint"), &cmd[1]);
    jnt_wheel_vel.registerHandle(handle_wheel_right);

    // registe wheel interface to robot
    this->registerInterface(&jnt_wheel_state);
    this->registerInterface(&jnt_wheel_vel);

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

    // start serial msg recv and proc thread
    thread_recv_msg = std::thread(&OrangeRobot::recv_msg_forever, this);
    thread_proc_msg = std::thread(&OrangeRobot::proc_msg_forever, this);

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
    delete stm32_bridge;
}

void OrangeRobot::recv_msg_forever() {
    while (true) {
        // recv a msg
        STM32Message msg = stm32_bridge->parse();

        // push to queue
        mutex_msg.lock();
        queue_msg.push(std::move(msg));
        cv_msg.notify_one();
        mutex_msg.unlock();
    }
}

void OrangeRobot::proc_msg_forever() {

    while (true) {
        // get a msg from queue
        std::unique_lock<std::mutex> lock(mutex_msg);
        // use cv_msg.wait() to wait msg ready
        if (queue_msg.size() <= 0) {
            cv_msg.wait(lock, [this]() { return queue_msg.size() > 0; });
        }

        STM32Message msg(std::move(queue_msg.front()));
        queue_msg.pop();
        lock.unlock();

        // lock sensor_data
        mutex_buf.lock();

        // proccess msg
        if (msg.type == STM32MessageType::MY_MESSAGE_NONE) {
            count_error++;
        } else {
            // handler msg
            // ROS_DEBUG_STREAM(msg);
            count_frame++;
            if (msg.type == STM32MessageType::MY_MESSAGE_TIMESTAMP) {
                // timestamp msg
                ros::Time stm32_time =
                    ros::Time(*reinterpret_cast<double *>(msg.content.data()));
                ros::Time ros_time = chrono_to_ros(msg.tp);
                ros::Duration new_diff = ros_time - stm32_time;
                if (std::fabs(new_diff.toSec() - stm32_time_diff.toSec()) >
                    0.1) {
                    if (stm32_time_diff.isZero()) {
                        ROS_INFO_STREAM("Time synced with STM32");
                    } else {
                        ROS_WARN_STREAM("Time resynced with STM32");
                    }
                }
                stm32_time_diff = new_diff;
            } else if (msg.type == STM32MessageType::MY_MESSAGE_STRING) {
                // string msg
                ROS_INFO_STREAM(reinterpret_cast<char *>(msg.content.data()));
            } else if (msg.type == STM32MessageType::MY_MESSAGE_MOTOR) {
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
            } else if (msg.type == STM32MessageType::MY_MESSAGE_ENCODER) {
                // encoder msg
                const Encoder_DataTypedef *dat =
                    reinterpret_cast<Encoder_DataTypedef *>(msg.content.data());

                // check time sync
                ros::Duration time_diff =
                    chrono_to_ros(msg.tp) -
                    (ros::Time(dat->timestamp) + stm32_time_diff);
                if (std::fabs(time_diff.toSec()) > 0.1) {
                    ROS_WARN_STREAM("Encoder timestamp miss match, " << msg);
                    continue;
                }

                // check encoder data
                if (dat->period <= 0) {
                    ROS_WARN_STREAM("Encoder period <= 0, " << msg);
                    continue;
                }
                std_msgs::Float64MultiArray wheel_msg;
                wheel_msg.data.resize(2, 0);
                wheel_msg.data[0] = dat->delta_left / dat->period;
                wheel_msg.data[1] = dat->delta_right / dat->period;
                if (fabs(wheel_msg.data[0]) > 2 ||
                    fabs(wheel_msg.data[1]) > 2) {
                    ROS_WARN_STREAM("Encoder data incorrect, " << msg);
                    continue;
                }

                buf_pos[0] = dat->distance_left / wheel_radius;
                buf_pos[1] = dat->distance_right / wheel_radius;
                ros::Time new_time = ros::Time::now();
                double dt;
                if (old_time.isZero()) {
                    dt = static_cast<double>(dat->period);
                } else {
                    dt = (new_time - old_time).toSec();
                }

                for (size_t i = 0; i < 2; i++) {
                    // calc wheel speed m/s
                    if (old_time.isZero()) {
                        // in first loop, use period from remote
                        buf_vel[i] = wheel_msg.data[i];
                    } else {
                        // has old pos
                        buf_vel[i] =
                            (buf_pos[i] - old_pos[i]) * wheel_radius / dt;
                    }
                    old_pos[i] = buf_pos[i];

                    // PID control
                    std::string status = pid[i].update(
                        buf_cmd[i] * wheel_radius, buf_vel[i], dt);
                    // ROS_INFO_STREAM(i << "->" <<
                    //                 buf_cmd[i] << " " <<
                    //                 buf_vel[i] << " " <<
                    //                 dt << " " <<
                    //                 pid[i].output);
                    if (status.size()) {
                        ROS_WARN_STREAM_NAMED("Motor PID", status);
                    }
                }
                old_time = new_time;
                // send motor duty cmd
                stm32_bridge->sendMsgMotor(static_cast<float>(pid[0].output),
                                           static_cast<float>(pid[1].output));

                has_new_encoder = chrono_to_ros(msg.tp);

                pub_wheel.publish(wheel_msg);

            } else if (msg.type == STM32MessageType::MY_MESSAGE_IMU) {
                // encoder msg
                const MPU_DataTypedef *dat =
                    reinterpret_cast<MPU_DataTypedef *>(msg.content.data());

                // check time sync
                ros::Duration time_diff =
                    chrono_to_ros(msg.tp) -
                    (ros::Time(dat->timestamp) + stm32_time_diff);
                if (std::fabs(time_diff.toSec()) > 0.1) {
                    ROS_WARN_STREAM("IMU timestamp miss match, " << msg);
                    continue;
                }

                imu_data.frame_id = "imu_link";
                buf_gyro[0] = dat->gyro_x;
                buf_gyro[1] = dat->gyro_y;
                buf_gyro[2] = dat->gyro_z;
                buf_accel[0] = dat->accel_x;
                buf_accel[0] = dat->accel_y;
                buf_accel[0] = dat->accel_z;

                has_new_imu = chrono_to_ros(msg.tp);
            }
        }
        if (!has_new_encoder.isZero() && !has_new_imu.isZero() &&
            std::fabs((has_new_encoder - has_new_imu).toSec()) < 0.1) {
            data_ready = true;
            cv_data_ready.notify_one();
        }
        mutex_buf.unlock();
    }
}

/* wait and read sensor data from hardware */
void OrangeRobot::read() {
    // wait for new sensor data (will lock mutex_buf)
    std::unique_lock<std::mutex> lock(mutex_buf);
    // if data not ready, use cv to wait for data ready
    if (!data_ready) {
        // cv.wait() will unlock mutex_buf when waiting
        // and lock mutex_buf when finish
        cv_data_ready.wait(lock, [this]() { return data_ready; });
    }

    std::copy(std::begin(buf_gyro), std::end(buf_gyro), std::begin(gyro));
    std::copy(std::begin(buf_accel), std::end(buf_accel), std::begin(accel));
    std::copy(std::begin(buf_vel), std::end(buf_vel), std::begin(vel));
    std::copy(std::begin(buf_pos), std::end(buf_pos), std::begin(pos));
    std::copy(std::begin(buf_eff), std::end(buf_eff), std::begin(eff));

    ros::Time mean_update_time =
        has_new_imu +
        ros::Duration((has_new_encoder - has_new_imu).toSec() / 2);

    data_ready = false;
    has_new_encoder.fromSec(0);
    has_new_imu.fromSec(0);

    if (update_time.isZero()) {
        // in first loop, period is 5ms
        update_period = ros::Duration(ros::Rate(200));
    } else {
        update_period = ros::Duration(mean_update_time - update_time);
    }
    update_time = mean_update_time;

    // unlock mutex_buf automately when return
}

/* wrtie control data to hardware */
void OrangeRobot::write() {
    // lock buf
    std::lock_guard<std::mutex> lock_buf(mutex_buf);

    std::copy(std::begin(cmd), std::end(cmd), std::begin(buf_cmd));

    // unlock mutex_buf automately when return
}
