#include "Kinematics.h"
#include <Arduino.h>

/**
 * @brief 限制数值在指定范围内
 */
float constrainValue(float value, float minValue, float maxValue)
{
    if (value > maxValue)
        return maxValue;
    if (value < minValue)
        return minValue;
    return value;
}

/**
 * @brief 安全计算平方根，避免负数输入
 */
float safe_sqrt(float x)
{
    return x >= 0.0f ? sqrtf(x) : 0.0f; // 记得有个快速开平方的式子，可优化
}
/**
 * @brief 安全除法，避免除以零
 */
float safe_div(float num, float denom)
{
    if (fabs(denom) < 1e-6f)
        denom = (denom < 0 ? -1e-6f : 1e-6f); // 防止分母0
    return num / denom;
}
/**
 * @brief 运动学逆解函数
 * @param LeftTarget 左腿目标位置
 * @param RightTarget 右腿目标位置
 * @param jointAngles 计算得到的关节角度
 */
void inverseKinematics(Node *LeftTarget, Node *RightTarget, JointAngles *jointAngles)
{
    float alpha1, alpha2, beta1, beta2;
    LeftTarget->y = constrainValue(LeftTarget->y, 110, 300);
    RightTarget->y = constrainValue(RightTarget->y, 110, 300);

    // 右腿逆解运算
    // 对 \alpha 进行计算所需参数
    float aRight = 2 * RightTarget->x * L1;                                                               // 参数q
    float bRight = 2 * RightTarget->y * L1;                                                               // 参数b
    float cRight = RightTarget->x * RightTarget->x + RightTarget->y * RightTarget->y + L1 * L1 - L2 * L2; // 参数c
    // 对 \beta 进行计算所需参数
    float dRight = 2 * L4 * (RightTarget->x - L5);                                                                        // 参数d
    float eRight = 2 * L4 * RightTarget->y;                                                                               // 参数e
    float fRight = ((RightTarget->x - L5) * (RightTarget->x - L5) + L4 * L4 + RightTarget->y * RightTarget->y - L3 * L3); // 参数f
    // 求解 \alpha 角
    float sqrtAlpha = safe_sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight)); // 求根公式中开根号部分
    float denomAlpha = aRight + cRight;                                                     // 求根公式分母
    alpha1 = 2 * atan(safe_div(bRight + sqrtAlpha, denomAlpha));
    alpha2 = 2 * atan(safe_div(bRight - sqrtAlpha, denomAlpha));
    // 求解 \beta 角
    float sqrtBeta = safe_sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight)); // 求根公式中开根号部分
    float denomBeta = dRight + fRight;                                                   // 求根公式分母
    beta1 = 2 * atan(safe_div(eRight + sqrtBeta, denomBeta));
    beta2 = 2 * atan(safe_div(eRight - sqrtBeta, denomBeta));
    // 角度转为正
    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);
    // 右腿角度赋值（加健壮性检查）
    if (!isnan(alpha1) && !isinf(alpha1) && alpha1 >= 0 && alpha1 <= PI)
        jointAngles->alphaRight = alpha1;
    else if (!isnan(alpha2) && !isinf(alpha2) && alpha2 >= 0 && alpha2 <= PI)
        jointAngles->alphaRight = alpha2;
    if (!isnan(beta1) && !isinf(beta1) && beta1 >= 0 && beta1 <= PI / 2)
        jointAngles->betaRight = beta1;
    else if (!isnan(beta2) && !isinf(beta2) && beta2 >= 0 && beta2 <= PI / 2)
        jointAngles->betaRight = beta2;

    // 左腿逆解运算
    // 对 \alpha 进行计算所需参数
    float aLeft = 2 * LeftTarget->x * L1;
    float bLeft = 2 * LeftTarget->y * L1;
    float cLeft = LeftTarget->x * LeftTarget->x + LeftTarget->y * LeftTarget->y + L1 * L1 - L2 * L2;
    // 对 \beta 进行计算所需参数
    float dLeft = 2 * L4 * (LeftTarget->x - L5);
    float eLeft = 2 * L4 * LeftTarget->y;
    float fLeft = ((LeftTarget->x - L5) * (LeftTarget->x - L5) + L4 * L4 + LeftTarget->y * LeftTarget->y - L3 * L3);
    // 求解 \alpha 角
    float sqrtAlphaL = safe_sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft)); // 求根公式中开根号部分
    float denomAlphaL = aLeft + cLeft;                                                 // 求根公式分母
    alpha1 = 2 * atan(safe_div(bLeft + sqrtAlphaL, denomAlphaL));
    alpha2 = 2 * atan(safe_div(bLeft - sqrtAlphaL, denomAlphaL));
    // 求解 \beta 角
    float sqrtBetaL = safe_sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft)); // 求根公式中开根号部分
    float denomBetaL = dLeft + fLeft;                                               // 求根公式分母
    beta1 = 2 * atan(safe_div(eLeft + sqrtBetaL, denomBetaL));
    beta2 = 2 * atan(safe_div(eLeft - sqrtBetaL, denomBetaL));
    // 角度转为正
    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);
    // 左腿角度赋值（加健壮性检查）
    if (!isnan(alpha1) && !isinf(alpha1) && alpha1 >= 0 && alpha1 <= PI)
        jointAngles->alphaLeft = alpha1;
    else if (!isnan(alpha2) && !isinf(alpha2) && alpha2 >= 0 && alpha2 <= PI)
        jointAngles->alphaLeft = alpha2;
    if (!isnan(beta1) && !isinf(beta1) && beta1 >= 0 && beta1 <= PI / 2)
        jointAngles->betaLeft = beta1;
    else if (!isnan(beta2) && !isinf(beta2) && beta2 >= 0 && beta2 <= PI / 2)
        jointAngles->betaLeft = beta2;
}

/**
 * @brief 运动学正解函数
 * @param LeftTarget 左腿位置输出
 * @param RightTarget 右腿位置输出
 * @param jointAngles 输入的关节角度(提前处理好，弧度制)
 */
void forwardKinematics(Node *LeftTarget, Node *RightTarget, JointAngles *jointAngles)
{
    float theta1, theta2;
    Node A, C;

    // 右腿解算 
    //计算A,C点坐标
    A.x = L1 * cosf(jointAngles->alphaRight);
    A.y = L1 * sinf(jointAngles->alphaRight);
    C.x = L5 + L4 * cosf(jointAngles->betaRight);
    C.y = L4 * sinf(jointAngles->betaRight);
    //计算对应参数
    float aRight = 2 * (C.x - A.x) * L2;//参数a
    float bRight = 2 * (C.y - A.y) * L2;//参数b
    float lRight = sqrt((C.x - A.x) * (C.x - A.x) + (C.y - A.y) * (C.y - A.y)); // a点与c点的距离
    float cRight = L2 * L2 + lRight * lRight - L3 * L3;//参数c
    //解算theta1，theta2
    theta1 = 2 * atan((bRight + sqrt((bRight * bRight) + (aRight * aRight) - (cRight * cRight))) / (aRight + cRight));
    theta2 = 2 * atan((bRight - sqrt((bRight * bRight) + (aRight * aRight) - (cRight * cRight))) / (aRight + cRight));
    // 角度解算范围限制
    theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2 * PI);
    theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2 * PI);
    if (theta1 >= PI / 2)
    {
        jointAngles->thetaRight1 = theta2;
    }
    else
    {
        jointAngles->thetaRight1 = theta1;
    }
    //计算脚端位置
    RightTarget->x = A.x + L2 * cos(jointAngles->thetaRight1);
    RightTarget->y = A.y + L2 * sin(jointAngles->thetaRight1);
    RightTarget->polarAngle = atan2f(RightTarget->y, RightTarget->x - 0.5f * L5);//极角
    RightTarget->radius = sqrtf((RightTarget->x - 0.5f * L5) * (RightTarget->x - 0.5f * L5) + RightTarget->y * RightTarget->y);//腿长

    // 左腿解算
    A.x = L1 * cosf(jointAngles->alphaLeft);
    A.y = L1 * sinf(jointAngles->alphaLeft);
    C.x = L5 + L4 * cosf(jointAngles->betaLeft);
    C.y = L4 * sinf(jointAngles->betaLeft);
    //计算对应参数
    float aLeft = 2 * (C.x - A.x) * L2;//参数a
    float bLeft = 2 * (C.y - A.y) * L2;//参数b
    float lLeft = sqrt((C.x - A.x) * (C.x - A.x) + (C.y - A.y) * (C.y - A.y));// a点与c点的距离
    float cLeft = L2 * L2 + lLeft * lLeft - L3 * L3;//参数c
    //解算theta1，theta2
    theta1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    theta2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    // 角度解算范围限制
    theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2 * PI);
    theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2 * PI);
    if (theta1 >= PI / 2)
    {
        jointAngles->thetaLeft1 = theta2;
    }
    else
    {
        jointAngles->thetaLeft1 = theta1;
    }
    //计算脚端位置
    LeftTarget->x = A.x + L2 * cos(jointAngles->thetaLeft1);
    LeftTarget->y = A.y + L2 * sin(jointAngles->thetaLeft1);
    LeftTarget->polarAngle = atan2f(LeftTarget->y, LeftTarget->x - 0.5f * L5);//极角
    LeftTarget->radius = sqrtf((LeftTarget->x - 0.5f * L5) * (LeftTarget->x - 0.5f * L5) + LeftTarget->y * LeftTarget->y);//腿长
}

/**
 * @brief 右腿运动学正解函数
 * @param RightPosFK 右腿位置输出
 * @param jointAngles 输入的关节角度(提前处理好，弧度制)
 */
void forwardKinematicsRight(Node *RightPosFK, JointAngles *jointAngles)
{
    float theta1, theta2;
    Node A, C;

    A.x = L1 * cosf(jointAngles->alphaRight);
    A.y = L1 * sinf(jointAngles->alphaRight);
    C.x = L5 + L4 * cosf(jointAngles->betaRight);
    C.y = L4 * sinf(jointAngles->betaRight);

    float aRight = 2 * (C.x - A.x) * L2;
    float bRight = 2 * (C.y - A.y) * L2;
    float lRight = sqrt((C.x - A.x) * (C.x - A.x) + (C.y - A.y) * (C.y - A.y)); // a点与b点的距离
    float cRight = L2 * L2 + lRight * lRight - L3 * L3;

    theta1 = 2 * atan((bRight + sqrt((bRight * bRight) + (aRight * aRight) - (cRight * cRight))) / (aRight + cRight));
    theta2 = 2 * atan((bRight - sqrt((bRight * bRight) + (aRight * aRight) - (cRight * cRight))) / (aRight + cRight));

    // 角度解算范围限制
    theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2 * PI);
    theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2 * PI);

    if (theta1 >= PI / 2)
    {
        jointAngles->thetaRight1 = theta2;
    }
    else
    {
        jointAngles->thetaRight1 = theta1;
    }

    RightPosFK->x = A.x + L2 * cos(jointAngles->thetaRight1);
    RightPosFK->y = A.y + L2 * sin(jointAngles->thetaRight1);
    RightPosFK->polarAngle = atan2f(RightPosFK->y, RightPosFK->x - 0.5f * L5);
    RightPosFK->radius = sqrtf((RightPosFK->x - 0.5f * L5) * (RightPosFK->x - 0.5f * L5) + RightPosFK->y * RightPosFK->y);
}
/**
 * @brief 左腿运动学正解函数
 * @param LeftPosFK 左腿位置输出
 * @param jointAngles 输入的关节角度(提前处理好，弧度制)
 */

void forwardKinematicsLeft(Node *LeftPosFK, JointAngles *jointAngles)
{
    float theta1, theta2;
    Node A, C;

    A.x = L1 * cosf(jointAngles->alphaLeft);
    A.y = L1 * sinf(jointAngles->alphaLeft);
    C.x = L5 + L4 * cosf(jointAngles->betaLeft);
    C.y = L4 * sinf(jointAngles->betaLeft);

    float aLeft = 2 * (C.x - A.x) * L2;
    float bLeft = 2 * (C.y - A.y) * L2;
    float lLeft = sqrt((C.x - A.x) * (C.x - A.x) + (C.y - A.y) * (C.y - A.y));
    float cLeft = L2 * L2 + lLeft * lLeft - L3 * L3;

    theta1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    theta2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));

    // 角度解算范围限制
    theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2 * PI);
    theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2 * PI);

    if (theta1 >= PI / 2)
    {
        jointAngles->thetaLeft1 = theta2;
    }
    else
    {
        jointAngles->thetaLeft1 = theta1;
    }

    LeftPosFK->x = A.x + L2 * cos(jointAngles->thetaLeft1);
    LeftPosFK->y = A.y + L2 * sin(jointAngles->thetaLeft1);
    LeftPosFK->polarAngle = atan2f(LeftPosFK->y, LeftPosFK->x - 0.5f * L5);
    LeftPosFK->radius = sqrtf((LeftPosFK->x - 0.5f * L5) * (LeftPosFK->x - 0.5f * L5) + LeftPosFK->y * LeftPosFK->y);
}
/**
 * @brief 计算右腿脚端速度
 * @param xdot 输出的脚端x方向速度
 * @param ydot 输出的脚端y方向速度
 * @param jointAngles 当前关节角度
 * @param alphaMotorvel α电机角速度
 * @param betaMotorvel β电机角速度
 */
void footRightVelocity(float *xdot, float *ydot, JointAngles *jointAngles, float alphaMotorvel, float betaMotorvel)
{
    float alpha = jointAngles->alphaRight;
    float beta = jointAngles->betaRight;
    float theta = jointAngles->thetaRight1;
    float alphaVel = alphaMotorvel;
    float betaVel = betaMotorvel;

    float Ax = L1 * cosf(alpha);
    float Ay = L1 * sinf(alpha);
    float Cx = L5 + L4 * cosf(beta);
    float Cy = L4 * sinf(beta);

    float dx = Cx - Ax;
    float dy = Cy - Ay;
    float a = 2 * L2 * dx;
    float b = 2 * L2 * dy;

    float dAx = -L1 * sinf(alpha) * alphaVel;
    float dAy = L1 * cosf(alpha) * alphaVel;
    float dCx = -L4 * sinf(beta) * betaVel;
    float dCy = L4 * cosf(beta) * betaVel;

    float ddx = dCx - dAx;
    float ddy = dCy - dAy;

    float adot = 2.0f * L2 * ddx;
    float bdot = 2.0f * L2 * ddy;
    float cdot = 2.0f * (dx * ddx + dy * ddy);

    float ct = cosf(theta), st = sinf(theta);
    float thetaDot = 0.0f;
    if (fabsf(b * ct - a * st) > 1e-6f)
    {
        thetaDot = (cdot - adot * ct - bdot * st) / (b * ct - a * st);
    }
    // 脚端速度
    *xdot = -L1 * sinf(alpha) * alphaVel - L2 * sinf(theta) * thetaDot;
    *ydot = L1 * cosf(alpha) * alphaVel + L2 * cosf(theta) * thetaDot;
}
/**
 * @brief 计算左腿脚端速度
 * @param xdot 输出的脚端x方向速度
 * @param ydot 输出的脚端y方向速度
 * @param jointAngles 当前关节角度
 * @param alphaMotorvel α电机角速度
 * @param betaMotorvel β电机角速度
 */
void footLeftVelocity(float *xdot, float *ydot, JointAngles *jointAngles, float alphaMotorvel, float betaMotorvel)
{
    float alpha = jointAngles->alphaLeft;
    float beta = jointAngles->betaLeft;
    float theta = jointAngles->thetaLeft1;
    float alphaVel = alphaMotorvel;
    float betaVel = betaMotorvel;

    float Ax = L1 * cosf(alpha);
    float Ay = L1 * sinf(alpha);
    float Cx = L5 + L4 * cosf(beta);
    float Cy = L4 * sinf(beta);

    float dx = Cx - Ax;
    float dy = Cy - Ay;
    float a = 2 * L2 * dx;
    float b = 2 * L2 * dy;

    float dAx = -L1 * sinf(alpha) * alphaVel;
    float dAy = L1 * cosf(alpha) * alphaVel;
    float dCx = -L4 * sinf(beta) * betaVel;
    float dCy = L4 * cosf(beta) * betaVel;

    float ddx = dCx - dAx;
    float ddy = dCy - dAy;

    float adot = 2.0f * L2 * ddx;
    float bdot = 2.0f * L2 * ddy;
    float cdot = 2.0f * (dx * ddx + dy * ddy);

    float ct = cosf(theta), st = sinf(theta);
    float thetaDot = 0.0f;
    if (fabsf(b * ct - a * st) > 1e-6f)
    {
        thetaDot = (cdot - adot * ct - bdot * st) / (b * ct - a * st);
    }

    // 脚端速度
    *xdot = -L1 * sinf(alpha) * alphaVel - L2 * sinf(theta) * thetaDot;
    *ydot = L1 * cosf(alpha) * alphaVel + L2 * cosf(theta) * thetaDot;
}

