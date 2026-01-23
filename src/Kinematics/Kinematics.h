#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// 正逆解控制参数
#define L1 150
#define L2 250
#define L3 250
#define L4 150
#define L5 108
#define L6 330

typedef struct {
    float x;
    float y;
    
    float polarAngle;//极角
    float radius;    //腿长
    float wheel_Torque;//轮子力矩

    float frontTorque;//前腿力矩
    float backTorque; //后腿力矩
} Node;




typedef struct 
{
    float alphaLeft;   // 左腿α角度
    float betaLeft;     // 左腿β角度
    float alphaRight;  // 右腿α角度
    float betaRight;   // 右腿β角度
    //theta2未使用(在推导过程中被消去)
    float thetaRight1; // 右腿θ1角度
    float thetaLeft1;  // 左腿θ1角度
}JointAngles;



float constrainValue(float value, float minValue, float maxValue);
void inverseKinematics(Node * LeftTarget, Node * RightTarget ,JointAngles * jointAngles);
void forwardKinematics(Node *LeftTarget, Node *RightTarget, JointAngles *jointAngles);
void forwardKinematicsRight(Node *RightTarget, JointAngles *jointAngles);
void forwardKinematicsLeft(Node *LeftTarget, JointAngles *jointAngles);
void footRightVelocity(float *xdot,float *ydot,JointAngles *jointAngles,float alphaMotorvel, float betaMotorvel);
void footLeftVelocity(float *xdot,float *ydot,JointAngles *jointAngles,float alphaMotorvel, float betaMotorvel);
#endif

