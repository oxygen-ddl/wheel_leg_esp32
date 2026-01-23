#ifndef _PID_H
#define _PID_H
#include "Arduino.h"

class PID_Controller {
    public: 
    
    PID_Controller(float kp, float ki, float kd, float dt, float min_output, float max_output);
    float PID_cal(float setpoint, float measured_value);
    void reset();

    private:
    float kp_;
    float ki_;
    float kd_;
    float dt_;
    float min_output_;
    float max_output_;
    float integral_;
    float previous_error_;
};

#endif
