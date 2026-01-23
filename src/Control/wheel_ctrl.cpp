#include "wheel_ctrl.h"
#include "Kinematics/Kinematics.h"
#include "Control/bipedal_data.h"

Wheel_TYPE wheel_data;
uint32_t last_time = 0;
uint32_t now_time = 0;
float motor1_vel = 0.0f; // 右轮电机角速度，单位rad/s,目前不知道，轮子速度从哪里获取
float motor2_vel = 0.0f; // 左轮电机角速度，单位rad/s

float roll ;
float pitch;
float yaw  ;
float gyroX;
float gyroY;
float gyroZ;

float forwardBackward = 0;// 前后移动控制量
float steering = 0; // 转向控制量
int speed_limit = 3; // 轮毂电机力矩限制

Node LeftPosFK;//左侧轮子
Node RightPosFK;//右侧轮子


// 限幅函数
float clampToRange(float value, float minVal, float maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

// 更新轮子参数
void update_wheel_data()
{
    
    now_time = millis(); // 获取当前时间，单位为毫秒，如果用micros()，约71.6分钟后会溢出
    if (last_time == 0)
    {
        last_time = now_time;
    }
    float delta_t = (now_time - last_time) * 1e-3f; // 转换为秒
    last_time = now_time;
    // 计算轮子速度和位移
    wheel_data.right_wheel_vel = motor1_vel * radius; // 右轮线速度，单位mm/s
    wheel_data.left_wheel_vel = motor2_vel * radius;  // 左轮线速度，单位mm/s
    wheel_data.right_wheel_x += wheel_data.right_wheel_vel * delta_t; // 右轮x方向位移，单位mm
    wheel_data.left_wheel_x += wheel_data.left_wheel_vel * delta_t;   // 左轮x方向位移，单位mm
    wheel_data.wheel_average_vel = (wheel_data.right_wheel_vel + wheel_data.left_wheel_vel) / 2.0f; // 平均线速度，单位mm/s
    wheel_data.wheel_average_x += wheel_data.wheel_average_vel * delta_t; // 平均x方向位移，单位mm
}

// 由matlab拟合得到的轮子控制参数K的计算函数
float calc_K1(float h) { return 0.000652 * h * h - 0.207928 * h + 1.964124; }
float calc_K2(float h) { return 0.000045 * h * h - 0.027259 * h + 1.626482; }
float calc_K3(float h) { return -0.000000 * h * h + 0.000000 * h + -0.000000; }
float calc_K4(float h) { return 0.000006 * h * h - 0.003406 * h - 4.027118; }

// 控制底部轮子霍尔电机,在pitch中加入lqr控制
void wheel_control(Node *RightPosFK, Node *LeftPosFK)
{
    update_wheel_data();

    float K1Right = calc_K1(RightPosFK->y);
    float K2Right = calc_K2(RightPosFK->y);
    float K3Right = calc_K3(RightPosFK->y);
    float K4Right = calc_K4(RightPosFK->y);

    float K1Left = calc_K1(LeftPosFK->y);
    float K2Left = calc_K2(LeftPosFK->y);
    float K3Left = calc_K3(LeftPosFK->y);
    float K4Left = calc_K4(LeftPosFK->y);

    float right_wheel_pitch_Torque = K1Right * pitch *PI / 180 + K2Right * (gyroY)*PI / 180 + K3Right *wheel_data.wheel_average_x  + K4Right * wheel_data.wheel_average_vel;
    float left_wheel_pitch_Torque  = K1Left *  pitch *PI / 180 + K2Left *  (gyroY)*PI / 180 + K3Left * wheel_data.wheel_average_x  + K4Left *  wheel_data.wheel_average_vel;
    RightPosFK->wheel_Torque = clampToRange(-right_wheel_pitch_Torque -  steering -  forwardBackward, -speed_limit, speed_limit);
    LeftPosFK->wheel_Torque  = clampToRange(-left_wheel_pitch_Torque  +  steering -  forwardBackward, -speed_limit, speed_limit);
}   

