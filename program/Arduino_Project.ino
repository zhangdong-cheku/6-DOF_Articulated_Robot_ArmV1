// ============================================================================
// 文件：Pos_Current_Velocity.ino
// 功能：FOC系统主程序 - 位置-速度-电流三环控制
// 说明：实现基于AS5600传感器的位置闭环、速度闭环和电流闭环控制
// ============================================================================

// ============================================================================
// 头文件包含
// ============================================================================
#include "FOC.h"           //!< FOC核心库头文件 - 包含FOC算法实现
#include "Ble_Handler.h"   //!< BLE通信处理头文件 - 支持无线控制功能

// ============================================================================
// 系统配置参数
// ============================================================================

// 传感器和电机参数配置
int Sensor_DIR = -1;  //!< 传感器方向：-1表示反向，1表示正向
                      //!< 用于校正传感器读数与电机实际旋转方向的关系

int Motor_PP = 7;     //!< 电机极对数：7对极（14极电机）
                      //!< 定义电机的磁极数量，用于电角度计算

// 备用配置（注释状态）
//int Motor_PP = 14;     //!< 电机极对数：14对极（28极电机）- 备用配置

// ============================================================================
// PID控制器限幅参数
// ============================================================================

// 位置环PID限幅参数
int angle_PID_limit = 60;  //!< 角度PID输出限幅：70度/秒
                          //!< 限制位置环控制器的最大输出速度

// 速度环PID限幅参数  
float vel_PID_limit = 6.5;  //!< 速度PID输出限幅：6.5安培（电流）
                           //!< 限制速度环控制器的最大输出电流

// 备用配置（注释状态）
//int angle_PID_limit = 5;  //!< 角度PID输出限幅：5度/秒 - 备用配置

// ============================================================================
// 函数：setup
// 功能：系统初始化函数
// 说明：在系统启动时执行一次，完成硬件和软件的初始化
// ============================================================================
void setup() {
  // 串口通信初始化
  Serial.begin(115200);  //!< 初始化串口通信，波特率115200

  // 电机使能控制
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  // FOC系统参数配置
  setPowerSupplyVoltage(15.0);

  calibrateSensor(Motor_PP, Sensor_DIR);

  // BLE通信初始化
  initBLEServer();

  // 启动帮助提示
  Serial.println("=== 串口控制帮助 ===");
  Serial.println("输入格式：");
  Serial.println("  RUN | MODE RUN | MODE=RUN — 切换运行模式");
  Serial.println("  STANDBY | MODE STANDBY | MODE=STANDBY — 切换待机模式");
  Serial.println("  OUT_DEG=<度数> — 设置输出轴角度（度）");
  Serial.println("  <弧度值> — 设置电机轴目标角（弧度）");
  Serial.println("  HELP 或 ? — 显示本帮助");
  Serial.println("提示：GEAR_RATIO 设置在 FOC.h 中。");
}

// ============================================================================
// 全局变量定义
// ============================================================================
int count = 0;  //!< 循环计数器 - 可用于调试或定时任务

// ============================================================================
// 函数：loop
// 功能：主循环函数
// 说明：系统主循环，持续执行控制算法和通信处理
// ============================================================================
void loop() {
  // 通信处理
  BLE_Server_Loop();

  // 模式分支：运行模式执行完整控制；待机仅刷新传感器
  if (control_mode == MODE_RUN) {
    runFOC();
    configureAnglePID(1, 0, 0, 10000, angle_PID_limit);
    configureVelocityPID(0.02, 1, 0, 10000, vel_PID_limit);
    configureCurrentPID(5, 200, 0, 10000);
    setMotorVelocityWithAngle(getSerialMotorTarget());
  } else {
    S0.Sensor_update();
  }

  // 串口命令处理
  readSerialCommand();

  // 每1秒输出一次 motor_target（调试），并显示输出轴角度
  static unsigned long last_log_ms = 0;
  unsigned long now_ms = millis();
  if (now_ms - last_log_ms >= 1000) {
    float target_out_deg = motor_target * 180.0f / PI / GEAR_RATIO;
    float actual_rad = getMotorAngle();
    float actual_out_deg = actual_rad * 180.0f / PI / GEAR_RATIO;
    Serial.printf("[DEBUG] mode=%s, target=%.4f rad, out_deg=%.2f°, actual=%.4f rad, actual_out_deg=%.2f°\n",
                  (control_mode == MODE_RUN ? "RUN" : "STANDBY"),
                  motor_target, target_out_deg,
                  actual_rad, actual_out_deg);
    last_log_ms = now_ms;
  }
}
