#include <Arduino.h> 
#include "InlineCurrent.h"

// ============================================================================
// 硬件配置常量定义
// 说明：定义ADC采样相关的硬件参数
// ============================================================================

#define _ADC_VOLTAGE 3.3f            // ADC参考电压（伏特）- ESP32的ADC工作电压
#define _ADC_RESOLUTION 4095.0f      // ADC分辨率 - ESP32的12位ADC（0-4095）

// ADC计数到电压转换比率：电压 = 计数值 × 转换比率
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

// 特殊值定义，用于检测引脚是否设置
#define NOT_SET -12345.0
#define _isset(a) ( (a) != (NOT_SET) )

// ============================================================================
// 类：CurrSense
// 功能：电流传感器类，负责电流信号的采集和处理
// ============================================================================

// ============================================================================
// 构造函数：CurrSense
// 功能：根据电机编号初始化电流传感器参数
// 参数：Mot_Num - 电机编号（0或1）
// 说明：支持多电机系统，为不同电机配置不同的ADC引脚
// ============================================================================
CurrSense::CurrSense(int Mot_Num)
{
  // 电机0的配置
  if(Mot_Num==0)
  {
    pinA = 39;      // A相电流检测ADC引脚（ESP32 GPIO39）
    pinB = 36;      // B相电流检测ADC引脚（ESP32 GPIO36）
    // pinC未设置，使用两相电流检测（第三相通过计算得到）
    
    _shunt_resistor = 0.01;  // 分流电阻值：0.01Ω（10mΩ）
    amp_gain  = 50;          // 运算放大器增益：50倍
    
    // 计算电压到电流的转换比率
    // 公式：电流 = 电压 / (分流电阻 × 放大器增益)
    volts_to_amps_ratio = 1.0f /_shunt_resistor / amp_gain;
    
    // 设置各相的增益系数（三相相同）
    gain_a = volts_to_amps_ratio;  // A相增益
    gain_b = volts_to_amps_ratio;  // B相增益
    gain_c = volts_to_amps_ratio;  // C相增益
  }
  
  // 电机1的配置（备用电机接口）
  if(Mot_Num==1)
  {
    pinA = 35;      // A相电流检测ADC引脚（ESP32 GPIO35）
    pinB = 34;      // B相电流检测ADC引脚（ESP32 GPIO34）
    // pinC未设置
    
    _shunt_resistor = 0.01;  // 分流电阻值：0.01Ω
    amp_gain  = 50;          // 运算放大器增益：50倍
    
    // 电压到电流转换比率计算
    volts_to_amps_ratio = 1.0f /_shunt_resistor / amp_gain;
    
    // 各相增益设置
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
  }
}

// ============================================================================
// 函数：readADCVoltageInline
// 功能：读取指定ADC引脚的电压值
// 参数：pinA - ADC引脚编号
// 返回值：转换后的电压值（伏特）
// 说明：将ADC原始计数值转换为实际电压值
// ============================================================================
float CurrSense::readADCVoltageInline(const int pinA){
  // 读取ADC原始值（0-4095）
  uint32_t raw_adc = analogRead(pinA);
  
  // ADC计数值转换为电压值：电压 = 计数值 × 转换比率
  return raw_adc * _ADC_CONV;
}

// ============================================================================
// 函数：configureADCInline
// 功能：配置ADC引脚的输入模式
// 参数：pinA, pinB, pinC - 三相ADC引脚
// 说明：将ADC引脚设置为输入模式，准备进行电压采样
// ============================================================================
void CurrSense::configureADCInline(const int pinA,const int pinB, const int pinC){
  pinMode(pinA, INPUT);  // 设置A相引脚为输入模式
  pinMode(pinB, INPUT);  // 设置B相引脚为输入模式
  
  // 如果C相引脚已设置，也配置为输入模式
  if( _isset(pinC) ) pinMode(pinC, INPUT);
}

// ============================================================================
// 函数：calibrateOffsets
// 功能：电流传感器零点校准
// 说明：在电机不通电时测量ADC偏移电压，用于后续电流计算中的误差补偿
// ============================================================================
void CurrSense::calibrateOffsets(){
    const int calibration_rounds = 1000;  // 校准采样次数

    // 初始化偏移量累加器
    offset_ia = 0;  // A相偏移量累加
    offset_ib = 0;  // B相偏移量累加
    offset_ic = 0;  // C相偏移量累加
    
    // 进行多次采样求平均，提高校准精度
    for (int i = 0; i < calibration_rounds; i++) {
        offset_ia += readADCVoltageInline(pinA);  // 累加A相采样值
        offset_ib += readADCVoltageInline(pinB);  // 累加B相采样值
        
        // 如果C相引脚已设置，也进行采样
        if(_isset(pinC)) offset_ic += readADCVoltageInline(pinC);
        
        delay(1);  // 短暂延时，避免采样过于密集
    }
    
    // 计算各相的平均偏移电压（零点误差）
    offset_ia = offset_ia / calibration_rounds;  // A相平均偏移
    offset_ib = offset_ib / calibration_rounds;  // B相平均偏移
    
    // 如果C相引脚已设置，计算C相平均偏移
    if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

// ============================================================================
// 函数：init
// 功能：电流传感器初始化
// 说明：完整的电流传感器初始化流程，包括硬件配置和软件校准
// ============================================================================
void CurrSense::init(){
    // 第一步：配置ADC引脚为输入模式
    configureADCInline(pinA,pinB,pinC);
    
    // 第二步：执行零点校准（需要在电机不通电时进行）
    calibrateOffsets();
}

// ============================================================================
// 函数：getPhaseCurrents
// 功能：读取三相电流值
// 说明：实时读取并计算三相电流，应用零点补偿和增益校正
// ============================================================================
void CurrSense::getPhaseCurrents(){
    // A相电流计算：
    // 电流 = (测量电压 - 偏移电压) × 增益系数
    current_a = (readADCVoltageInline(pinA) - offset_ia)*gain_a;
    
    // B相电流计算：
    current_b = (readADCVoltageInline(pinB) - offset_ib)*gain_b;
    
    // C相电流处理：
    // 如果C相引脚未设置，电流设为0（两相检测系统）
    // 如果C相引脚已设置，进行正常计算
    current_c = (!_isset(pinC)) ? 0 : (readADCVoltageInline(pinC) - offset_ic)*gain_c;
}