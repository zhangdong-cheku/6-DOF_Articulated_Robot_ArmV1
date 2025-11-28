#include "FOC.h"

// ============================================================================
// 全局变量定义区
// 说明：这些变量在整个FOC系统中共享使用，用于存储系统状态和控制参数
// ============================================================================

// 电源相关变量
float voltage_power_supply;  // 电源电压值（单位：伏特），在系统初始化时设置

// FOC变换过程中的中间电压变量
float Ualpha = 0;  // α轴电压分量（帕克逆变换输出）
float Ubeta = 0;   // β轴电压分量（帕克逆变换输出）
float Ua = 0;      // A相电压值（克拉克逆变换输出）
float Ub = 0;      // B相电压值（克拉克逆变换输出）
float Uc = 0;      // C相电压值（克拉克逆变换输出）

// 电机参数和状态变量
float zero_electric_angle = 0;  // 零电角度偏移量，在校准过程中确定
int PP = 1;                     // 电机极对数（Pole Pairs），默认值为1
int DIR = 1;                    // 电机旋转方向，1为正转，-1为反转

// PWM引脚定义
int pwmA = 32;  // A相PWM输出引脚（ESP32 GPIO32）
int pwmB = 33;  // B相PWM输出引脚（ESP32 GPIO33）
int pwmC = 25;  // C相PWM输出引脚（ESP32 GPIO25）

// 控制目标变量
float motor_target = 0.0;  // 电机目标位置（弧度），来自串口或BLE命令
// 新增：控制模式（默认运行）
int control_mode = MODE_RUN;

// ============================================================================
// 全局对象定义区
// 说明：这些对象封装了FOC系统的各种功能模块
// ============================================================================

// 滤波器对象
LowPassFilter M0_Vel_Flt = LowPassFilter(0.01);   // 速度环低通滤波器，截止频率0.01Hz
LowPassFilter M0_Curr_Flt = LowPassFilter(0.05);  // 电流环低通滤波器，截止频率0.05Hz

// PID控制器对象（三环控制结构）
PIDController vel_loop_M0 = PIDController{
    .P = 2,        // 速度环比例增益
    .I = 0,        // 速度环积分增益
    .D = 0,        // 速度环微分增益
    .ramp = 100000, // 输出变化率限制（防止突变）
    .limit = voltage_power_supply/2  // 输出限幅（电源电压的一半）
};

PIDController angle_loop_M0 = PIDController{
    .P = 2,        // 位置环比例增益
    .I = 0,        // 位置环积分增益
    .D = 0,        // 位置环微分增益
    .ramp = 100000, // 输出变化率限制
    .limit = 100   // 位置环输出限幅（100度/秒）
};

PIDController current_loop_M0 = PIDController{
    .P = 1.2,      // 电流环比例增益
    .I = 0,        // 电流环积分增益
    .D = 0,        // 电流环微分增益
    .ramp = 100000, // 输出变化率限制
    .limit = 12.6  // 电流环输出限幅（12.6A，基于电源电压计算）
};

// 传感器对象
Sensor_AS5600 S0 = Sensor_AS5600(0);  // AS5600磁编码器对象，I2C地址0
TwoWire S0_I2C = TwoWire(0);          // I2C总线对象，使用Wire0

// 电流传感器对象
CurrSense CS_M0 = CurrSense(0);       // 电流传感器对象，用于测量电机相电流