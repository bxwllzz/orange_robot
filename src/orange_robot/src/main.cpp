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
#include <controller_manager/controller_manager.h>

// #include "motor_pid.hpp"
#include "orangerobot_hw.h"
#include "stm32_bridge.hpp"

using namespace LZZ;

int main(int argc, char **argv) {

    ros::init(argc, argv, "orange_robot");
    ros::NodeHandle nh;

    OrangeRobot robot;
    robot.init(nh, nh);

    controller_manager::ControllerManager cm(&robot, nh);

    while (ros::ok()) {
        robot.read();
        cm.update(robot.get_time(), robot.get_period());
        robot.write();
    }

    return 0;
}
