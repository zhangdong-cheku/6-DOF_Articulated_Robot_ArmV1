#ifndef DENG_FOC_H
#define DENG_FOC_H

#include <Arduino.h>
#include "AS5600.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "InlineCurrent.h"
#include "Ble_Handler.h"

// 宏定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _3PI_2 4.71238898038f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f

//#define GEAR_RATIO 225.0f
#define GEAR_RATIO 100.0f
//#define GEAR_RATIO 15.0f
static const float I_MAX_CMD = 6.5f;

// 新增：运行模式常量
#define MODE_STANDBY 0
#define MODE_RUN 1

// 全局变量声明
extern float voltage_power_supply;
extern float Ualpha, Ubeta, Ua, Ub, Uc;
extern float zero_electric_angle;
extern int PP, DIR;
extern int pwmA, pwmB, pwmC;
extern float motor_target;
// 新增：控制模式
extern int control_mode;

// 全局对象声明
extern LowPassFilter M0_Vel_Flt;
extern LowPassFilter M0_Curr_Flt;
extern PIDController vel_loop_M0;
extern PIDController angle_loop_M0;
extern PIDController current_loop_M0;
extern Sensor_AS5600 S0;
extern TwoWire S0_I2C;
extern CurrSense CS_M0;

// 核心算法函数声明
float normalizeAngle(float angle);
void setPwm(float Ua, float Ub, float Uc);
void setTorque(float Uq, float angle_el);
void setPowerSupplyVoltage(float power_supply);
float electricalAngle();  
void calibrateSensor(int _PP, int _DIR);

// 传感器函数声明
float getMotorAngle();
float getMotorVelocity();
float calculateIqId(float current_a, float current_b, float angle_el);
float getMotorCurrent();

// PID控制函数声明
void configureVelocityPID(float P, float I, float D, float ramp, float limit);
void configureAnglePID(float P, float I, float D, float ramp, float limit);
void configureCurrentPID(float P, float I, float D, float ramp);
float calculateVelocityPID(float error);
float calculateAnglePID(float error);

// 控制接口函数声明
void setMotorTorque(float Target);
void setMotorVelocityWithAngle(float Target);
void runFOC();

// 新增：模式切换接口
void enterStandby();
void enterRun();

// 通信函数声明
String readSerialCommand();
float getSerialMotorTarget();

#endif
