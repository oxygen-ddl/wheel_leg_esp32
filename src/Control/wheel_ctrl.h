#ifndef WHEEL_CTRL_H
#define WHEEL_CTRL_H

#include "Arduino.h"
#include "../Kinematics/Kinematics.h"
typedef struct 
{
    float right_wheel_vel;//右轮线速度
    float left_wheel_vel;//左轮线速度
    float right_wheel_x;//右轮x方向位移
    float left_wheel_x;//左轮x方向位移
    float wheel_average_vel;//平均线速度
    float wheel_average_x;//平均x方向位移
}Wheel_TYPE;

extern Wheel_TYPE wheel_data;
extern Node LeftPosFK;//左侧轮子
extern Node RightPosFK;//右侧轮子
extern float motor1_vel; // 右轮电机角速度，单位rad/s
extern float motor2_vel; // 左轮电机角速度，单位rad/s
extern float forwardBackward;// 前后移动控制量
extern float steering; // 转向控制量

extern float roll ;
extern float pitch;
extern float yaw  ;
extern float gyroX;
extern float gyroY;
extern float gyroZ;
void wheel_control(Node *RightPosFK, Node *LeftPosFK);

#endif
