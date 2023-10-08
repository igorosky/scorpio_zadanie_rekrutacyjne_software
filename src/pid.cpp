#include "pid.hpp"
#include <algorithm>

// #define DEBUG
#ifdef DEBUG
#include <ros/ros.h>
#endif

Pid::Pid(float _pf, float _if, float _df) : p_factor(_pf), i_factor(_if), d_factor(_df), previous_error(0), target_value(0) { }

void Pid::set_proportion_factor(float v) {
    p_factor = v;
}

void Pid::set_integral_factor(float v) {
    i_factor = v;
}

void Pid::set_differentation_factor(float v) {
    d_factor = v;
}

void Pid::set_target_value(float v) {
    previous_error = 0;
    target_value = v;
}

void Pid::set_target_value_with_default(float v, float int_val) {
    previous_error = 0;
    target_value = v;
    integral = int_val;
}

int8_t Pid::calculate(const float val, const float time) {
    const float error = target_value - val;
    float pid = error*p_factor;
    integral += error*time*i_factor;
    #ifdef DEBUG
    ROS_INFO("Error: %f", error);
    ROS_INFO("Proportion: %f", pid);
    ROS_INFO("Integral: %f", integral);
    #endif
    pid += integral;
    const float dif = d_factor*(error - previous_error)/time;
    #ifdef DEBUG
    ROS_INFO("Differentiation: %f", dif);
    #endif
    pid += dif;
    previous_error = error;
    #ifdef DEBUG
    ROS_INFO("Pid: %f", pid);
    #endif
    pid = std::max(std::min(pid, (float)100), (float)-100);
    return (int8_t)pid;
}

float Pid::get_target_value() const {
    return target_value;
}
