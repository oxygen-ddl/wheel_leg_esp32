#ifndef MITCTRL_H
#define MITCTRL_H

#include "Arduino.h"
#include "MIT.h"
#include "Kinematics/Kinematics.h"

#define SEND_INTERVAL 1 // 限制发送频率，单位为毫秒
extern float Am_kp;

void CAN_Control(Node LeftPos, Node RightPos);
void startMotor(int motorIndex);
void posInit();
void enableMotor();

#endif
