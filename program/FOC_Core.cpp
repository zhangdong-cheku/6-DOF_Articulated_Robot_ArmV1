#include "FOC.h"
#include "Ble_Handler.h"

// ============================================================================
// 函数：normalizeAngle
// 功能：角度归一化处理
// 参数：angle - 输入角度（弧度）
// 返回值：归一化到[0, 2π)范围内的角度
// 说明：将任意角度值转换为0到2π之间的等效角度，便于三角函数计算
// ============================================================================
float normalizeAngle(float angle) {
    // 使用模运算将角度限制在2π周期内
    float a = fmod(angle, 2*PI);
    // 处理负角度情况，转换为正角度
    return a >= 0 ? a : (a + 2*PI);
}

// ============================================================================
// 函数：setPwm
// 功能：三相PWM输出控制
// 参数：Ua, Ub, Uc - 三相电压值
// 说明：将三相电压转换为PWM占空比并输出到电机驱动器
// ============================================================================
void setPwm(float Ua, float Ub, float Uc) {
    // 电压限幅：确保电压值在电源电压范围内
    Ua = _constrain(Ua, 0.0f, voltage_power_supply);
    Ub = _constrain(Ub, 0.0f, voltage_power_supply);
    Uc = _constrain(Uc, 0.0f, voltage_power_supply);
    
    // 电压转占空比：计算每相的PWM占空比（0-1范围）
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);
    
    // PWM输出：将占空比转换为8位PWM值（0-255）并输出
    ledcWrite(0, dc_a*255);  // A相PWM输出
    ledcWrite(1, dc_b*255);  // B相PWM输出
    ledcWrite(2, dc_c*255);  // C相PWM输出
}

// ============================================================================
// 函数：setTorque
// 功能：FOC核心算法 - 力矩控制
// 参数：Uq - q轴电压（力矩分量），angle_el - 电角度（弧度）
// 说明：实现FOC算法的核心部分，包括帕克逆变换和克拉克逆变换
// ============================================================================
//void setTorque(float Uq, float angle_el) {
//    // q轴电压限幅：限制在±电源电压/2范围内
//    Uq = _constrain(Uq, -(voltage_power_supply)/2, (voltage_power_supply)/2);
//    float Ud = 0;  // d轴电压设为0（磁场定向控制，d轴不产生力矩）
//    
//    // 电角度归一化处理
//    angle_el = normalizeAngle(angle_el);
//    
//    // 帕克逆变换（Park逆变换）：将dq坐标系转换为αβ坐标系
//    // Uα = -Uq * sin(θ) + Ud * cos(θ)
//    // Uβ = Uq * cos(θ) + Ud * sin(θ)
//    // 由于Ud=0，简化为：
//    Ualpha = -Uq*sin(angle_el);  // α轴电压分量
//    Ubeta = Uq*cos(angle_el);    // β轴电压分量
//    
//    // 克拉克逆变换（Clarke逆变换）：将αβ坐标系转换为三相ABC坐标系
//    // 标准三相逆变器电压公式：
//    Ua = Ualpha + voltage_power_supply/2;                    // A相电压
//    Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;  // B相电压
//    Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2; // C相电压
//    
//    // 调用PWM输出函数，将电压转换为实际PWM信号
//    setPwm(Ua, Ub, Uc);
//}

void setTorque(float Uq, float angle_el)
{
  if (Uq < 0)
    angle_el += _PI;
  Uq = abs(Uq);

  angle_el = normalizeAngle(angle_el + _PI_2);
  int sector = floor(angle_el / _PI_3) + 1;
  // calculate the duty cycles
  float T1 = _SQRT3 * sin(sector * _PI_3 - angle_el) * Uq / voltage_power_supply;
  float T2 = _SQRT3 * sin(angle_el - (sector - 1.0) * _PI_3) * Uq / voltage_power_supply;
  float T0 = 1 - T1 - T2;

  float Ta, Tb, Tc;
  switch (sector)
  {
  case 1:
    Ta = T1 + T2 + T0 / 2;
    Tb = T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 2:
    Ta = T1 + T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 3:
    Ta = T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T2 + T0 / 2;
    break;
  case 4:
    Ta = T0 / 2;
    Tb = T1 + T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 5:
    Ta = T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 6:
    Ta = T1 + T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T0 / 2;
    break;
  default:
    Ta = 0;
    Tb = 0;
    Tc = 0;
  }

  float Ua = Ta * voltage_power_supply;
  float Ub = Tb * voltage_power_supply;
  float Uc = Tc * voltage_power_supply;

  setPwm(Ua, Ub, Uc);
}

// ============================================================================
// 函数：electricalAngle
// 功能：计算电角度
// 返回值：归一化后的电角度（弧度）
// 说明：将机械角度转换为电角度，考虑极对数和旋转方向
// ============================================================================
float electricalAngle() {
    // 电角度 = 极对数 × 机械角度 × 方向 - 零电角度偏移
    // 公式：θ_elec = PP × θ_mech × DIR - θ_zero
    return normalizeAngle((float)(DIR * PP) * S0.getMechanicalAngle() - zero_electric_angle);
}

// ============================================================================
// 函数：setPowerSupplyVoltage
// 功能：系统硬件初始化
// 参数：power_supply - 电源电压值
// 说明：初始化PWM、编码器、电流传感器等硬件外设
// ============================================================================
void setPowerSupplyVoltage(float power_supply) {
    // 设置电源电压全局变量
    voltage_power_supply = power_supply;
    
    // PWM引脚初始化
    pinMode(pwmA, OUTPUT);  // A相PWM引脚
    pinMode(pwmB, OUTPUT);  // B相PWM引脚
    pinMode(pwmC, OUTPUT);  // C相PWM引脚
    
    // PWM通道配置：30kHz频率，8位分辨率
    ledcSetup(0, 30000, 8);  // 通道0：A相
    ledcSetup(1, 30000, 8);  // 通道1：B相
    ledcSetup(2, 30000, 8);  // 通道2：C相
    
    // PWM引脚与通道绑定
    ledcAttachPin(pwmA, 0);  // A相引脚绑定到通道0
    ledcAttachPin(pwmB, 1);  // B相引脚绑定到通道1
    ledcAttachPin(pwmC, 2);  // C相引脚绑定到通道2
    
    Serial.println("完成PWM初始化设置");

    // AS5600磁编码器初始化
    // I2C总线配置：SDA=19, SCL=18, 400kHz速率
    S0_I2C.begin(19, 18, 400000UL);
    S0.Sensor_init(&S0_I2C);  // 编码器传感器初始化
    Serial.println("编码器加载完毕");

    // 速度环PID控制器重新初始化
    vel_loop_M0 = PIDController{.P = 2, .I = 0, .D = 0, .ramp = 100000, .limit = voltage_power_supply/2};
    
    // 电流传感器初始化
    CS_M0.init();
}

// ============================================================================
// 函数：calibrateSensor
// 功能：传感器校准程序
// 参数：_PP - 电机极对数，_DIR - 旋转方向
// 说明：执行电机零电角度校准，确定磁场定向的基准位置
// ============================================================================
void calibrateSensor(int _PP, int _DIR) {
    // 设置电机参数
    PP = _PP;   // 极对数
    DIR = _DIR; // 旋转方向
    
    // 第一步：施加固定力矩使电机转到特定位置（3π/2位置）
    // 这个位置有助于确定零电角度
    setTorque(3, _3PI_2);
    delay(1000);  // 等待1秒让电机稳定
    
    // 第二步：更新编码器读数
    S0.Sensor_update();
    
    // 第三步：计算并保存零电角度
    // 零电角度是电机当前位置的电角度，作为后续计算的基准
    zero_electric_angle = electricalAngle();
    
    // 第四步：释放力矩，电机回到自由状态
    setTorque(0, _3PI_2);
    
    // 输出校准结果
    Serial.print("0电角度：");
    Serial.println(zero_electric_angle);
}

// 新增：进入待机模式（零扭矩、目标继承当前角）
void enterStandby() {
  control_mode = MODE_STANDBY;
  // 继承当前电机轴角，作为后续恢复运行时的初始目标
  motor_target = getMotorAngle();
  // 立即置零力矩，确保进入待机后无驱动电流
  setTorque(0, electricalAngle());
  Serial.println("[MODE] Enter STANDBY: zero torque, target=curr angle");
}

// 新增：进入运行模式（继承当前角为初始目标，避免跳变）
void enterRun() {
  control_mode = MODE_RUN;
  // 以当前电机轴角作为运行初始目标，避免从待机恢复出现位置跳变
  motor_target = getMotorAngle();
  Serial.println("[MODE] Enter RUN: inherit current angle as target");
  // 进入 RUN 时发送一次实际输出侧角度（度），用于反馈到客户端
  float angle_deg = getCurrentAngleDegrees();
  sendMultiStructAngleFeedback(angle_deg);
  char resp[64];
  snprintf(resp, sizeof(resp), "%d:FEEDBACK:%.2f", getMyDeviceID(), angle_deg);
  sendBLEResponse(resp);
}
