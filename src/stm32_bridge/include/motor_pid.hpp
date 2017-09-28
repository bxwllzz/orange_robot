/*
 * motor_pid.hpp
 *
 *  Created on: 2017年7月4日
 *      Author: hustac
 */

#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <iostream>
#include <chrono>

namespace LZZ {

class MotorPID {
public:
    double Kp;
    double Ki;
    double Kd;
    double cutoff_freq;
    double integral_limit;
    double output_limit;

    std::chrono::high_resolution_clock::time_point prev_time;
    double prev_error;
    double error_integral;

    double output_p;
    double output_i;
    double output_d;
    double output;

    MotorPID();
    std::string update(double desired, double real,
            std::chrono::high_resolution_clock::time_point tp);

};

}

#endif /* INCLUDE_MOTOR_PID_HPP_ */
