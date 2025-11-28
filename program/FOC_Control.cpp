#include "FOC.h"

// ============================================================================
// 函数：setMotorTorque
// 功能：力矩控制函数（电流环控制）
// 参数：Target - 目标电流值（力矩指令）
// 说明：这是最内层的电流环控制，直接控制电机的输出力矩
// ============================================================================
void setMotorTorque(float Target) {
    // 计算电流环PID输出：目标电流 - 实际测量电流
    float current_error = Target - getMotorCurrent();
    float pid_output = current_loop_M0(current_error);
    
    // 使用PID输出和电角度设置电机力矩
    setTorque(pid_output, electricalAngle());
}

// ============================================================================
// 函数：setMotorVelocityWithAngle
// 功能：位置-速度-电流三环控制（外环到内环的级联控制）
// 参数：Target - 目标位置（弧度）
// 说明：实现完整的三环控制策略：
//       位置环（外环）→ 速度环（中环）→ 电流环（内环）
// ============================================================================
void setMotorVelocityWithAngle(float Target) {
    // 1. 位置环控制：计算位置误差并转换为角度PID
    //    将弧度转换为角度（180/PI），计算位置误差
    float position_error = (Target - getMotorAngle()) * 180 / PI;
    float angle_pid_output = calculateAnglePID(position_error);
    
    // 2. 速度环控制：将位置环输出作为速度环的参考值
    //    速度环输入 = 位置环输出 - 实际速度
    float velocity_error = angle_pid_output - getMotorVelocity();
    float iq_ref = calculateVelocityPID(velocity_error);
    
    // 3. 电流限幅：限制q轴电流参考值在安全范围内
    iq_ref = _constrain(iq_ref, -I_MAX_CMD, I_MAX_CMD);
    
    // 4. 调用力矩控制函数（电流环）
    setMotorTorque(iq_ref);
}

// ============================================================================
// 函数：runFOC
// 功能：FOC主控制循环
// 说明：每个控制周期需要执行的核心任务
//       1. 更新传感器数据（角度）
//       2. 更新电流传感器数据
// ============================================================================
void runFOC() {
    // 更新磁编码器角度数据
    S0.Sensor_update();
    
    // 更新三相电流测量值
    CS_M0.getPhaseCurrents();
}

// ============================================================================
// 函数：readSerialCommand
// 功能：串口通信命令处理
// 返回值：接收到的完整命令字符串
// 说明：处理来自串口的控制命令，支持多字符命令的接收和解析
// ============================================================================
String readSerialCommand() {
    static String received_chars;  // 静态缓冲区，保存未完成的命令字符
    String command = "";           // 完整的命令字符串

    while (Serial.available()) {
        char inChar = (char)Serial.read();
        // 行结束（支持\n或\r）
        if (inChar == '\n' || inChar == '\r') {
            command = received_chars;   // 取一整行
            received_chars = "";       // 清空缓冲

            // 规范化命令：去空白、转大写
            String cmd = command;
            cmd.trim();
            if (cmd.length() == 0) return command; // 空行直接返回
            String upper = cmd;
            upper.toUpperCase();

            // 帮助提示：HELP / ? / H
            if (upper == "HELP" || upper == "?" || upper == "H") {
                Serial.println("=== 串口控制帮助 ===");
                Serial.println("输入格式：");
                Serial.println("  RUN | MODE RUN | MODE=RUN — 切换运行模式");
                Serial.println("  STANDBY | MODE STANDBY | MODE=STANDBY — 切换待机模式");
                Serial.println("  OUT_DEG=<度数> — 设置输出轴角度（度）");
                Serial.println("  <弧度值> — 设置电机轴目标角（弧度）");
                Serial.println("  HELP 或 ? — 显示本帮助");
                Serial.println("提示：GEAR_RATIO 设置在 FOC.h 中。");
                // 同步当前目标信息
                float out_deg_dbg = motor_target * 180.0f / PI / GEAR_RATIO;
                Serial.printf("[SERIAL] 当前目标: %.4f rad, out_deg=%.2f°\n", motor_target, out_deg_dbg);
                return command;
            }

            // 模式切换：RUN / STANDBY
            if (upper == "RUN" || upper == "MODE RUN" || upper == "MODE=RUN") {
                enterRun();
                Serial.println("[SERIAL] Mode -> RUN");
                float out_deg_dbg = motor_target * 180.0f / PI / GEAR_RATIO;
                Serial.printf("[SERIAL] target=%.4f rad, out_deg=%.2f°\n", motor_target, out_deg_dbg);
                return command;
            }
            if (upper == "STANDBY" || upper == "MODE STANDBY" || upper == "MODE=STANDBY") {
                enterStandby();
                Serial.println("[SERIAL] Mode -> STANDBY");
                float out_deg_dbg = motor_target * 180.0f / PI / GEAR_RATIO;
                Serial.printf("[SERIAL] target=%.4f rad, out_deg=%.2f°\n", motor_target, out_deg_dbg);
                return command;
            }

            // 目标角设置（输出角度，单位度）：OUT_DEG=xxx
            if (upper.startsWith("OUT_DEG=")) {
                int eq = upper.indexOf('=');
                float out_deg = cmd.substring(eq + 1).toFloat();
                motor_target = out_deg * GEAR_RATIO * (PI / 180.0f);
                Serial.printf("[SERIAL] out_deg %.2f° -> motor_target %.4f rad\n", out_deg, motor_target);
                // 再回显转换结果，保持一致的格式
                Serial.printf("[SERIAL] target=%.4f rad, out_deg=%.2f°\n", motor_target, out_deg);
                return command;
            }

            // 直接数字：按弧度设置 motor_target
            char c0 = upper.charAt(0);
            if (c0 == '-' || (c0 >= '0' && c0 <= '9')) {
                motor_target = cmd.toFloat();
                float out_deg_dbg = motor_target * 180.0f / PI / GEAR_RATIO;
                Serial.printf("[SERIAL] motor_target set to %.4f rad\n", motor_target);
                Serial.printf("[SERIAL] target=%.4f rad, out_deg=%.2f°\n", motor_target, out_deg_dbg);
                return command;
            }

            // 未识别命令，打印提示
            Serial.print("[SERIAL] Unknown command: ");
            Serial.println(cmd);
            Serial.println("输入 'HELP' 或 '?' 获取命令格式帮助");
        } else {
            // 累加字符
            received_chars += inChar;
        }
    }
    return command;  // 返回本轮解析到的完整命令（可能为空）
}

// ============================================================================
// 函数：getSerialMotorTarget
// 功能：BLE蓝牙目标值处理
// 返回值：处理后的电机目标位置（弧度）
// 说明：处理来自蓝牙的电机控制命令，支持角度到弧度的转换和去重处理
// ============================================================================
float getSerialMotorTarget() {
    static float last_target = 0.0f;  // 保存上一次的目标值，用于去重
    
    // 检查是否有新的BLE命令到达
    if (new_command) {
        new_command = false;  // 重置命令标志
        
        // BLE传输的是输出角度（度），需要转换为电机轴角度
        float out_deg = ble_motor_target;
        
        // 角度转换：输出角度 → 电机机械角度（弧度）
        // 考虑减速比和角度单位转换
        float motor_rad = out_deg * GEAR_RATIO * (PI / 180.0f);

        // 去重处理：只有当目标值变化超过阈值时才更新
        if (fabs(motor_rad - last_target) > 0.0001f) {
            last_target = motor_rad;     // 更新上一次目标值
            motor_target = motor_rad;    // 设置新的电机目标
            
            // 调试信息：显示转换过程和参数
            Serial.printf("[DEBUG] BLE输出角度 %.2f° -> 电机目标 %.4f rad (ratio=%g)\n",
                          out_deg, motor_rad, GEAR_RATIO);
        } else {
            // 目标值未变化时的调试信息
            Serial.printf("[DEBUG] BLE目标未改变: 输出角度 %.2f°\n", out_deg);
        }
    }
    
    // 返回当前电机目标位置
    return motor_target;
}