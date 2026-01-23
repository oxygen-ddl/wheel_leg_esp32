#include "motor_ctrl.h"
#include "Control/wheel_ctrl.h"
#include "Kinematics/Kinematics.h"
#include "bipedal_data.h"
#include "MIT/MIT.h"
#include "pid.h"
#include "Control/bipedal_data.h"

IKparam IKParam;
// 左腿相关运动学基本参数
motionControlParams LeftMotionControlParams;
// 右腿相关运动学基本参数
motionControlParams RightMotionControlParams;
motorsparam motorsParam;



// ROLL控制
float roll_target = 0.0f;   // 目标滚转角度，单位deg
float roll_banlance = 0.0f; // 滚转角度平衡控制，调节轮腿高度，单位mm
//高度控制
float startHeight = 150.0f; // 初始高度，单位mm
float remoteHeight = 0.0f;  // 遥控高度调节，单位mm
float jump_value = 0.0f;    // 跳跃高度调节，单位mm
//速度控制
float x_vel = 0.0f;         // X方向速度，单位mm/s

PID_Controller roll_pid(0.5f, 0.0f, 0.1f, 0.01, -10.0f, 10.0f); // 滚转角度PID控制器
PID_Controller height_pid(0.5f, 0.0f, 0.06f, 0.01, -30.0f, 100.0f); // 高度控制PID控制器
PID_Controller x_vel_pid(0.2f, 0.0f, 0.05f, 0.01, -20.0f, 20.0f);   // X方向速度PID控制器

float targetHeight = 0.0f;
void calRightVMC(Node *RightPosFK, JointAngles *jointAngles)
{
    float j11, j12, j21, j22;// 雅可比矩阵元素
    float theta2;
    float fG = 25; // 重力前馈

    //footRightVelocity(&xdot, &ydot, jointAngles, devicesState[0].vel, devicesState[2].vel);// 计算脚端速度
    forwardKinematicsRight(RightPosFK, jointAngles);// 计算脚端位置

    // 计算Y方向力
    float height_output = height_pid.PID_cal(targetHeight, RightPosFK->y);//高度调整
    float roll_output = roll_pid.PID_cal(roll_target, roll); // 角度值的偏差值
    // 将roll角度转化到腿部高度调整
    roll_banlance = sin((roll_output * 3.14) / 180) * (LeftMotionControlParams.robotl) / 2; 
    float Fy = height_output + roll_banlance + fG; //Y方向力

    //计算x方向力
    float x_vel_output = x_vel_pid.PID_cal(0.0f, wheel_data.wheel_average_vel);
    float Fx = x_vel_output; //X方向力

    // 计算雅可比矩阵
     theta2 = acos((RightPosFK->x - (L5 + L4 * cos(jointAngles->betaRight))) / L3);
    j11 = (L1 * sin(theta2) * sin(jointAngles->alphaRight - jointAngles->thetaRight1)) / sin(jointAngles->thetaRight1 - theta2);
    j12 = (L4 * sin(jointAngles->thetaRight1) * sin(theta2 - jointAngles->betaRight)) / sin(jointAngles->thetaRight1 - theta2);
    j21 = -(L1 * cos(theta2) * sin(jointAngles->alphaRight - jointAngles->thetaRight1)) / sin(jointAngles->thetaRight1 - theta2);
    j22 = -(L4 * cos(jointAngles->thetaRight1) * sin(theta2 - jointAngles->betaRight)) / sin(jointAngles->thetaRight1 - theta2);

    RightPosFK->backTorque  = (j11 * Fx + j21 * Fy) / 1000.0f;
    RightPosFK->frontTorque = (j12 * Fx + j22 * Fy) / 1000.0f;
}

void calLeftVMC(Node *LeftPosFK, JointAngles *jointAngles)
{
    float j11, j12, j21, j22;// 雅可比矩阵元素
    float theta2;
    float fG = 25; // 重力前馈

    //footLeftVelocity(&xdot, &ydot, jointAngles, devicesState[1].vel, devicesState[3].vel);// 计算脚端速度
    forwardKinematicsLeft(LeftPosFK, jointAngles);// 计算脚端位置

    // 计算Y方向力
    float height_output = height_pid.PID_cal(targetHeight, LeftPosFK->y);//高度调整
    float roll_output = roll_pid.PID_cal(roll_target, roll); // 角度值的偏差值
    // 将roll角度转化到腿部高度调整
    roll_banlance = sin((roll_output * 3.14) / 180) * (LeftMotionControlParams.robotl) / 2; 
    float Fy = height_output - roll_banlance + fG; //Y方向力

    //计算x方向力
    // float x_vel_output = x_vel_pid.PID_cal(0.0f, wheel_data.wheel_average_vel);
    // float Fx = x_vel_output; //X方向力
    //先不采用pid得到x方向力
    float Fx = forwardBackward; 
    // 计算雅可比矩阵
     theta2 = acos((LeftPosFK->x - (L5 + L4 * cos(jointAngles->betaLeft))) / L3);
    j11 = (L1 * sin(theta2) * sin(jointAngles->alphaLeft - jointAngles->thetaLeft1)) / sin(jointAngles->thetaLeft1 - theta2);
    j12 = (L4 * sin(jointAngles->thetaLeft1) * sin(theta2 - jointAngles->betaLeft)) / sin(jointAngles->thetaLeft1 - theta2);
    j21 = -(L1 * cos(theta2) * sin(jointAngles->alphaLeft - jointAngles->thetaLeft1)) / sin(jointAngles->thetaLeft1 - theta2);
    j22 = -(L4 * cos(jointAngles->thetaLeft1) * sin(theta2 - jointAngles->betaLeft)) / sin(jointAngles->thetaLeft1 - theta2);

    LeftPosFK->backTorque  = (j11 * Fx + j21 * Fy) / 1000.0f;// 单位转换为Nm
    LeftPosFK->frontTorque = (j12 * Fx + j22 * Fy) / 1000.0f;// 单位转换为Nm
}
