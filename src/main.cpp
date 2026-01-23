#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//引入自己的库
#include "Kinematics/Kinematics.h"
#include "MPU6050/MPU6050.h"
#include "Control/motor_ctrl.h"
#include "Control/wheel_ctrl.h"
#include "Control/bipedal_data.h"
#include "MIT/MIT.h"
#include "MIT/MITCtrl.h"
#include "SF_Motor/Motor.h"//驱动轮子电机的库，未提供具体通讯协议
#include "Remote/remote_udp.h"


// Wi-Fi 信息
const char* ssid = "HITWH-QD";
const char* password = "hgd123456";
RemoteUDP Remote(ssid, password, 8686);//接受udp数据需要确认数据ip来源

MPU6050 mpu6050 = MPU6050(Wire); // 实例化MPU6050
void Open_thread_function();
void update_jointAngles();
void remote_to_control();



JointAngles M_jointAngles;//关节角度
float up_start = 1.0f;//轮子电机使能标志


void setup()
{
  Wire.begin(1, 2, 400000UL);//初始化IIC
  Serial.begin(115200);//初始化调试串口
  mpu6050.begin(); //初始化MPU陀螺仪
  Open_thread_function();//启动线程,读取陀螺仪数据在另外一个核上运行
  Remote.begin(); // 初始化UDP通信,重启读取硬件串口，获取新ip地址
  CANInit(); // 初始化CAN
  enableMotor();//使能关节电机
  motorInit(); // 初始化轮子电机

}

void loop()
{
  Remote.handleIncomingPackets();//处理udp数据包
  Remote.udp_justfloat(2,motor1_vel,motor2_vel);//通过udp接收轮子电机速度指令,真的能有速度吗？
  //! 记得将角度填入M_jointAngles中的角度值
  recCANMessage(); // CAN接收函数
  CAN_Control(LeftPosFK,RightPosFK);      // CAN 关节电机控制函数
  wheel_control(&RightPosFK, &LeftPosFK); // 轮子lqr控制
  update_jointAngles(); // 更新关节角度值
  calRightVMC(&RightPosFK,&M_jointAngles);// 右腿VMC控制
  calLeftVMC(&LeftPosFK,&M_jointAngles);  // 左腿VMC控制
  sendMotorTargets( up_start*RightPosFK.wheel_Torque,  up_start*LeftPosFK.wheel_Torque); // 发送控制轮毂电机的目标值

}
// 陀螺仪数据读取
void IMUTask(void *pvParameters)
{
  while (true)
  {
    mpu6050.update();
    roll  = mpu6050.getAngleX();
    pitch = mpu6050.getAngleY();
    yaw   = mpu6050.getAngleZ();
    gyroX = mpu6050.getGyroX();
    gyroY = mpu6050.getGyroY();
    gyroZ = mpu6050.getGyroZ();
  }
}

// 启动线程
void Open_thread_function()
{
  // 陀螺仪读取任务进程
  xTaskCreatePinnedToCore(
      IMUTask,   // 任务函数
      "IMUTask", // 任务名称
      4096,      // 堆栈大小
      NULL,      // 传递的参数
      1,         // 任务优先级
      NULL,      // 任务句柄
      1          // 运行在核心 1
  );
}

void update_jointAngles()
{
  //右腿
   M_jointAngles.alphaRight = (3*PI/4) - devicesState[2].pos/8  ;
   M_jointAngles.betaRight  = (3*PI/4) - devicesState[0].pos/8 ;
   //左腿
   M_jointAngles.alphaLeft  = (3*PI/4) - devicesState[3].pos/8 ;
   M_jointAngles.betaLeft   = (3*PI/4) - devicesState[1].pos/8 ;
}
// 远程控制转化为运动控制参数
void remote_to_control()
{
    //globalData_1.value1值范围为-1000到1000
    if (fabs(globalData_1.value1) > 200)
    {
      steering = globalData_1.value1*0.0001;        // 转向控制量，最大为0.1Nm
    }
    else
    {
      steering = 0.0f;
    }

    if (fabs(globalData_1.value2) > 200)
    {
      forwardBackward = globalData_1.value2*0.1;              // 线速度，最大为100mm/s
    }
    else
    {
      forwardBackward = 0.0f;
    }

    if (fabs(globalData_2.value1)> 200)
    {
      targetHeight = 140.0f + globalData_2.value1*0.01; // 目标高度，单位mm
    }
    else
    {
      targetHeight = 140.0f;
    }
}