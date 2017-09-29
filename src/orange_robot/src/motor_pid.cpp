#include "motor_pid.hpp"

#include <ratio>

namespace LZZ {

MotorPID::MotorPID() {
    Kp = 5;
    Ki = 10;
    Kd = 0;
    cutoff_freq = 0;
    integral_limit = 0.05;
    output_limit = 1;

    prev_error = 0;
    error_integral = 0;

    output_p = 0;
    output_i = 0;
    output_d = 0;
    output = 0;
}

std::string MotorPID::update(double desired, double real,
                             double dt) {
    std::string status = "";

    // if zero point, clear integral
    if (desired == .0 && real == .0) {
        error_integral = 0;
    }

    // p
    double error = desired - real;
    output_p = Kp * error;

    // i
    error_integral += error * dt;
    if (error_integral > integral_limit) {
        error_integral = integral_limit;
        status = "integral reach upper limit";
    } else if (error_integral < -integral_limit) {
        error_integral = -integral_limit;
        status = "integral reach lower limit";
    }
    output_i = Ki * error_integral;

    // d
    double error_deriv = (error - prev_error) / dt;
    if (dt <= 0) {
        error_deriv = 0;
        status = "delta time <= 0";
    }
    output_d = Kd * error_deriv;

    // sum
    output = output_p + output_i + output_d;
    if (output > output_limit) {
        output = output_limit;
        status = "output reach upper limit";
    } else if (output < -output_limit) {
        output = -output_limit;
        status = "output reach lower limit";
    }

    prev_error = error;

    return status;
}
}
