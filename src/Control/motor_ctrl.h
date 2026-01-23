#ifndef _MOTOR_CTRL_H
#define _MOTOR_CTRL_H

#include "Arduino.h"
#include "Kinematics/Kinematics.h"
void calRightVMC(Node *RightPosFK, JointAngles *jointAngles);
void calLeftVMC(Node *LeftPosFK, JointAngles *jointAngles);

extern float targetHeight;
extern float roll_target;

#endif
