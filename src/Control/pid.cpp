#include "pid.h"
PID_Controller::PID_Controller(float kp, float ki, float kd, float dt, float min_output, float max_output)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    dt_ = dt;
    min_output_ = min_output;
    max_output_ = max_output;
    integral_ = 0.0f;
    previous_error_ = 0.0f;
}

float PID_Controller::PID_cal(float setpoint, float measured_value)
{
    float error = setpoint - measured_value;
    integral_ += error * dt_;
    float derivative = (error - previous_error_) / dt_;
    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    output = fmax(min_output_, fmin(max_output_, output));
    previous_error_ = error;
    return output;
}

void PID_Controller::reset()
{
    integral_ = 0.0f;
    previous_error_ = 0.0f;
}
