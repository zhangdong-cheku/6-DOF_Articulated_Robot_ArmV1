#include "Ble_Handler.h"
#include "FOC.h"
#include <BLE2902.h>

// ============================================================================
// 全局变量定义
// ============================================================================

// 电机控制目标值
float ble_motor_target = 0.0f;    //!< BLE接收到的电机目标值（角度/速度/电流）
bool new_command = false;         //!< 新命令标志，表示有新控制指令需要处理
uint8_t data_scale_type = 0;      //!< 数据类型标识：0=角度，1=速度，2=电流
uint8_t my_device_id = 1;         //!< 本设备ID，用于多设备系统区分

// BLE服务器相关全局变量
bool deviceConnected = false;     //!< 当前设备连接状态
bool oldDeviceConnected = false;  //!< 前次设备连接状态，用于状态变化检测
BLEServer* pServer = nullptr;     //!< BLE服务器对象指针
BLEService* pService = nullptr;    //!< BLE服务对象指针
BLECharacteristic* pTxCharacteristic = nullptr;  //!< 发送特征值对象指针
BLECharacteristic* pRxCharacteristic = nullptr;  //!< 接收特征值对象指针

// 最近一次 MULTI_STRUCT 解析结果
MultiStructParsed last_multi_struct_cmd = {};

// ============================================================================
// BLE UUID定义
// 说明：使用标准UUID格式，确保与客户端匹配
// ============================================================================
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"           //!< 服务UUID
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"  //!< 接收特征值UUID
#define CHARACTERISTIC_UUID_TX "6d68efe5-04b6-4a85-abc4-c2670b7bf7fd"  //!< 发送特征值UUID

// ============================================================================
// 函数：bleDebugPrint
// 功能：BLE调试信息输出函数
// 参数：message - 要输出的调试信息
// 说明：根据BLE_DEBUG宏控制是否输出调试信息
// ============================================================================
void bleDebugPrint(const char* message) {
#if BLE_DEBUG
    Serial.print("[BLE] ");
    Serial.println(message);
#endif
}

// ============================================================================
// 函数：getMyDeviceID
// 功能：获取本设备ID
// 返回值：本设备的唯一标识符
// 说明：用于多设备系统中区分不同电机控制器
// ============================================================================
uint8_t getMyDeviceID() {
    return my_device_id;
}

// ============================================================================
// 函数：floatToInt16
// 功能：浮点数转换为16位整数（带缩放）
// 参数：value - 输入浮点数值，scale - 缩放系数
// 返回值：缩放后的16位整数值
// 说明：用于数据压缩传输，支持溢出保护
// ============================================================================
int16_t floatToInt16(float value, float scale) {
    int32_t scaled_value = (int32_t)(value * scale);  // 缩放并转换为32位整数
    if (scaled_value > 32767) scaled_value = 32767;   // 正向溢出保护
    if (scaled_value < -32768) scaled_value = -32768; // 负向溢出保护
    return (int16_t)scaled_value;                     // 转换为16位整数
}

// ============================================================================
// 函数：int16ToFloat
// 功能：16位整数转换为浮点数（带缩放）
// 参数：value - 输入16位整数值，scale - 缩放系数
// 返回值：缩放后的浮点数值
// 说明：用于数据解压缩，还原原始浮点数值
// ============================================================================
float int16ToFloat(int16_t value, float scale) {
    return (float)value / scale;  // 缩放还原为浮点数
}

float getCurrentAngleDegrees() {
    // 获取电机侧机械角（弧度），按方向已由 getMotorAngle() 处理
    float angle_motor_rad = getMotorAngle();
    // 转换到输出侧角度（度）
    float angle_output_deg = (angle_motor_rad / GEAR_RATIO) * (180.0f / PI);
    return angle_output_deg;
}

void sendMultiStructAngleFeedback(float angle_deg) {
    if (!deviceConnected || pTxCharacteristic == nullptr) {
        bleDebugPrint("设备未连接或特征值为空，无法发送反馈");
        return;
    }
    // AA 55 03 DT COUNT | ID VH VL
    uint8_t buf[8];
    buf[0] = 0xAA;
    buf[1] = 0x55;
    buf[2] = PACKET_TYPE_MULTI_STRUCT;
    buf[3] = DATA_TYPE_ANGLE;
    buf[4] = 1; // COUNT
    buf[5] = getMyDeviceID();
    int16_t raw = floatToInt16(angle_deg, ANGLE_SCALE);
    buf[6] = (uint8_t)((raw >> 8) & 0xFF);
    buf[7] = (uint8_t)(raw & 0xFF);
    pTxCharacteristic->setValue(buf, sizeof(buf));
    pTxCharacteristic->notify();
    Serial.printf("[BLE响应] FEEDBACK: id=%d angle=%.2f deg raw=%d\n", getMyDeviceID(), angle_deg, (int)raw);
}

// ============================================================================
// 函数：parseDirectCommandData
// 功能：解析直接命令数据包
// 参数：data - 接收到的原始数据包
// 说明：支持多种数据包格式，包括单电机控制、多电机批量控制等
// ============================================================================
void parseDirectCommandData(const std::string& data) {
    char debugMsg[100];  // 调试信息缓冲区
    uint8_t my_id = getMyDeviceID();  // 获取本设备ID
    
    // ============================================================================
    // 第一步：调试信息输出和基本检查
    // ============================================================================
    
    // 增强调试信息：分隔线与包长度输出
    Serial.println("==========================================");
    Serial.printf("[BLE调试] 开始解析直接命令数据，长度: %d\n", data.length());
    
    // 打印原始数据的十六进制表示（方便调试）
    Serial.print("[BLE调试] 原始数据(HEX): ");
    for (int i = 0; i < data.length(); i++) {
        Serial.printf("%02X ", (uint8_t)data[i]);  // 把char强制为uint8_t再打印为两位16进制
    }
    Serial.println();
    
    // 基本长度检查：至少3字节（用以判断帧头或包类型）
    if (data.length() < 3) {
        snprintf(debugMsg, sizeof(debugMsg), "数据太短: %d字节", data.length());
        bleDebugPrint(debugMsg);
        Serial.printf("[BLE错误] 数据长度不足，需要至少3字节，实际: %d\n", data.length());
        return;
    }
    
    // ============================================================================
    // 第二步：帧头检测和包类型识别
    // ============================================================================
    
    // 帧头检测：看前两字节是否为0xAA 0x55，并且第三字节要是有效包类型
    bool has_frame_header = false;
    if (data.length() >= 2 && data[0] == 0xAA && data[1] == 0x55) {
        // 支持SINGLE/MULTI/MULTI_STRUCT三种类型
        if (data.length() >= 3 && (data[2] == PACKET_TYPE_SINGLE || data[2] == PACKET_TYPE_MULTI || data[2] == PACKET_TYPE_MULTI_STRUCT)) {
            has_frame_header = true;
            Serial.println("[BLE调试] 检测到有效帧头(AA 55)，跳过帧头解析");
        } else {
            Serial.println("[BLE调试] 检测到AA 55但包类型无效，按无帧头处理");
        }
    }
    
    // 解析包类型（如果存在帧头则跳过前两字节）
    uint8_t packet_type;
    if (has_frame_header) {
        packet_type = data[2];  // 帧头后面的第一个字节是包类型
    } else {
        packet_type = data[0];  // 无帧头则第1字节就是包类型
    }
    
    snprintf(debugMsg, sizeof(debugMsg), "直接数据包类型: 0x%02X, My ID: %d", packet_type, my_id);
    bleDebugPrint(debugMsg);
    Serial.printf("[BLE调试] 包类型: 0x%02X, 设备ID: %d\n", packet_type, my_id);
    
    // ============================================================================
    // 第三步：根据包类型进行不同处理
    // ============================================================================
    
    if (packet_type == PACKET_TYPE_SINGLE) {  // 单电机控制包
        // 支持两种单包格式：
        // - 有帧头的7字节：AA 55 01 DT ID VH VL
        // - 无帧头的6字节：    01 ID DT VH VL 00（这里按你的注释）
        int min_length = has_frame_header ? 7 : 6;
        if (data.length() < min_length) {
            bleDebugPrint("单电机控制包太短");
            Serial.printf("[BLE错误] 单电机包长度不足，需要%d字节，实际: %d\n", min_length, data.length());
            return;
        }
        
        // 根据是否有帧头计算每个字段的偏移
        int id_offset, type_offset, value_offset;
        if (has_frame_header) {
            // 7字节包格式：AA(0) 55(1) 01(2) DT(3) ID(4) VH(5) VL(6)
            type_offset = 3;  
            id_offset = 4;    
            value_offset = 5; 
        } else {
            // 6字节包格式假设：01(0) ID(1) DT(2) VH(3) VL(4) 00(5)
            id_offset = 1;    
            type_offset = 2;  
            value_offset = 3; 
        }
        
        uint8_t target_id = data[id_offset];      // 目标设备ID（字节）
        uint8_t data_type = data[type_offset];    // 数据类型（字节）
        
        // 如果目标ID不是本设备则忽略（并不回送）
        if (target_id != my_id) {
            snprintf(debugMsg, sizeof(debugMsg), "不是本设备的数据 (期望 %d, 收到 %d)", my_id, target_id);
            bleDebugPrint(debugMsg);
            Serial.printf("[BLE调试] 数据不是给本设备的，期望ID: %d, 收到ID: %d，直接返回不发送响应\n", my_id, target_id);
            new_command = false;
            return;  // 直接返回，不发送任何响应
        }
        
        // 反馈请求处理：DATA_TYPE_FEEDBACK_REQ => 返回当前角度并退出
        if (data_type == DATA_TYPE_FEEDBACK_REQ) {
            float cur_deg = getCurrentAngleDegrees();
            sendMultiStructAngleFeedback(cur_deg);
            char resp[64];
            snprintf(resp, sizeof(resp), "%d:FEEDBACK:%.2f", my_id, cur_deg);
            sendBLEResponse(resp);
            return;
        }

        // 记录调试信息
        snprintf(debugMsg, sizeof(debugMsg), "单电机控制 - 目标ID: %d, 数据类型: 0x%02X", target_id, data_type);
        bleDebugPrint(debugMsg);
        Serial.printf("[BLE调试] 单电机控制 - 目标ID: %d, 数据类型: 0x%02X\n", target_id, data_type);
        
        // 输出缩放系数（假设ANGLE_SCALE是宏或全局变量）
        Serial.printf("[BLE调试] 使用缩放系数: %.1f\n", ANGLE_SCALE);
        
        // 提取2字节的int16_t值（高字节在前）并转换为float（按ANGLE_SCALE）
        int16_t target_int;
        target_int = (int16_t)(((uint8_t)data[value_offset] << 8) | (uint8_t)data[value_offset + 1]);
        
        // 模式控制：DATA_TYPE_MODE（非零=RUN，零=STANDBY）
        if (data_type == DATA_TYPE_MODE) {
            if (target_int != 0) {
                enterRun();
                char resp[64];
                snprintf(resp, sizeof(resp), "%d:MODE:RUN", my_id);
                sendBLEResponse(resp);
                Serial.printf("[BLE调试] 设备%d模式切换: RUN\n", my_id);
            } else {
                enterStandby();
                char resp[64];
                snprintf(resp, sizeof(resp), "%d:MODE:STANDBY", my_id);
                sendBLEResponse(resp);
                Serial.printf("[BLE调试] 设备%d模式切换: STANDBY\n", my_id);
            }
            return;
        }
        
        float new_target = int16ToFloat(target_int, ANGLE_SCALE);
        if (fabs(new_target - ble_motor_target) > 0.001f) {
            ble_motor_target = new_target;
            new_command = true;
            Serial.printf("[BLE调试] 目标值改变: %.2f -> %.2f，设置new_command\n", ble_motor_target, new_target);
        } else {
            new_command = false;
            Serial.printf("[BLE调试] 目标值未改变: %.2f，不设置new_command\n", new_target);
        }
        
        snprintf(debugMsg, sizeof(debugMsg), "直接单电机控制接收: %.2f", ble_motor_target);
        bleDebugPrint(debugMsg);
        
        // 额外打印原始字节和解析结果，便于调试
        Serial.printf("[BLE调试] 直接控制原始字节: %02X %02X, 解析值: %d, 缩放后: %.2f\n",
                      (uint8_t)data[value_offset], (uint8_t)data[value_offset + 1], target_int, ble_motor_target);
 
        // 发送BLE确认响应（格式: "<id>:SINGLE:<value>"）
        char response[50];
        snprintf(response, sizeof(response), "%d:SINGLE:%.2f", my_id, ble_motor_target);
        sendBLEResponse(response);
        
    } else if (packet_type == PACKET_TYPE_MULTI) {   // 多电机批量控制包（切片/兼容旧版）
        Serial.printf("[BLE调试] 开始处理多电机包，长度: %d\n", data.length());
    
        // 计算DT偏移（有帧头AA 55时为3；无帧头时为1）
        int type_offset = has_frame_header ? 3 : 1;
        if ((int)data.length() <= type_offset) {
            Serial.printf("[BLE错误] 多电机包长度不足以包含数据类型，长度: %d\n", data.length());
            return;
        }
    
        // 数据类型与缩放
        uint8_t data_type = (uint8_t)data[type_offset];
        float scale = ANGLE_SCALE;
        if (data_type == DATA_TYPE_VELOCITY) {
            scale = VELOCITY_SCALE;
            data_scale_type = 1;
            Serial.printf("[BLE调试] 使用速度缩放系数: %.2f\n", scale);
        } else if (data_type == DATA_TYPE_CURRENT) {
            scale = 1000.0f;
            data_scale_type = 2;
            Serial.printf("[BLE调试] 使用电流缩放系数: %.2f\n", scale);
        } else {
            data_scale_type = 0;
            Serial.printf("[BLE调试] 使用角度缩放系数: %.2f\n", scale);
        }
    
        // 优先尝试"切片式MULTI"：AA 55 02 DT START_ID COUNT V(start)..V(end)
        if (has_frame_header && data.length() >= 6) {
            int start_id_offset   = type_offset + 1;  // 4
            int count_offset      = type_offset + 2;  // 5
            int data_start_offset = type_offset + 3;  // 6
    
            uint8_t start_id = (uint8_t)data[start_id_offset];
            uint8_t count    = (uint8_t)data[count_offset];
    
            bool ids_ok = (start_id >= 1 && start_id <= MAX_MOTORS && count >= 1);
            bool len_ok = ((int)data.length() == data_start_offset + count * 2);
    
            if (ids_ok && len_ok) {
                uint8_t my_id = getMyDeviceID();
                uint8_t end_id = start_id + count - 1;
                Serial.printf("[BLE调试] 多电机控制(切片) - DT=0x%02X, 范围: ID %d..%d\n", data_type, start_id, end_id);
    
                if (my_id < start_id || my_id > end_id) {
                    Serial.printf("[BLE调试] 本设备ID %d不在当前切片范围内，忽略\n", my_id);
                    return;
                }
    
                int index_in_slice = (my_id - start_id);  // 0-based
                int data_offset = data_start_offset + index_in_slice * 2;
                if (data_offset + 2 > data.length()) {
                    Serial.printf("[BLE错误] 数据偏移超出包长度，偏移: %d, 包长度: %d\n", data_offset, data.length());
                    return;
                }
    
                int16_t target_int = (int16_t)(((uint16_t)(uint8_t)data[data_offset] << 8) | (uint16_t)(uint8_t)data[data_offset + 1]);
                Serial.printf("[BLE调试] 设备%d原始字节: %02X %02X, 解析值: %d\n",
                              my_id, (uint8_t)data[data_offset], (uint8_t)data[data_offset + 1], target_int);
    
                // 模式控制：DATA_TYPE_MODE（非零=RUN，零=STANDBY）
                if (data_type == DATA_TYPE_MODE) {
                    if (target_int != 0) {
                        enterRun();
                        char response_mode[50];
                        snprintf(response_mode, sizeof(response_mode), "%d:MODE:RUN", my_id);
                        sendBLEResponse(response_mode);
                        Serial.printf("[BLE调试] 设备%d模式切换(切片): RUN\n", my_id);
                    } else {
                        enterStandby();
                        char response_mode[50];
                        snprintf(response_mode, sizeof(response_mode), "%d:MODE:STANDBY", my_id);
                        sendBLEResponse(response_mode);
                        Serial.printf("[BLE调试] 设备%d模式切换(切片): STANDBY\n", my_id);
                    }
                    return;
                }
                float new_target = int16ToFloat(target_int, scale);
                Serial.printf("[BLE调试] 设备%d缩放后目标值: %.2f\n", my_id, new_target);
    
                if (fabs(new_target - ble_motor_target) > 0.001f) {
                    ble_motor_target = new_target;
                    new_command = true;
                    Serial.printf("[BLE调试] 设备%d目标值改变: %.2f -> %.2f，设置new_command\n", my_id, ble_motor_target, new_target);
                } else {
                    new_command = false;
                    Serial.printf("[BLE调试] 设备%d目标值未改变: %.2f，不设置new_command\n", my_id, new_target);
                }
    
                char response[50];
                snprintf(response, sizeof(response), "%d:MULTI:%.2f", my_id, ble_motor_target);
                sendBLEResponse(response);
                Serial.printf("[BLE调试] 设备%d收到指令: 多电机控制(切片), 目标值: %.2f\n", my_id, ble_motor_target);
                return;
            }
        }
    
        // 兼容旧版"整包10台"格式：AA 55 02 DT V1..V10（总长度24字节）
        if (has_frame_header && data.length() == 24) {
            int data_start_offset = type_offset + 1;  // 4
            uint8_t my_id = getMyDeviceID();
    
            if (my_id < 1 || my_id > 10) {
                Serial.printf("[BLE调试] 旧版整包不包含设备%d的数据\n", my_id);
                return;
            }
    
            int idx = (my_id - 1);  // 1-based → 0-based
            int data_offset = data_start_offset + idx * 2;
            if (data_offset + 2 > data.length()) {
                Serial.printf("[BLE错误] 数据偏移超出包长度(旧版)，偏移: %d, 包长度: %d\n", data_offset, data.length());
                return;
            }
    
            int16_t target_int = (int16_t)(((uint16_t)(uint8_t)data[data_offset] << 8) | (uint16_t)(uint8_t)data[data_offset + 1]);
            Serial.printf("[BLE调试] 设备%d(旧版)原始字节: %02X %02X, 解析值: %d\n",
                          my_id, (uint8_t)data[data_offset], (uint8_t)data[data_offset + 1], target_int);
    
            // 模式控制：DATA_TYPE_MODE（非零=RUN，零=STANDBY）
            if (data_type == DATA_TYPE_MODE) {
                if (target_int != 0) {
                    enterRun();
                    char response_mode[50];
                    snprintf(response_mode, sizeof(response_mode), "%d:MODE:RUN", my_id);
                    sendBLEResponse(response_mode);
                    Serial.printf("[BLE调试] 设备%d模式切换(旧版整包): RUN\n", my_id);
                } else {
                    enterStandby();
                    char response_mode[50];
                    snprintf(response_mode, sizeof(response_mode), "%d:MODE:STANDBY", my_id);
                    sendBLEResponse(response_mode);
                    Serial.printf("[BLE调试] 设备%d模式切换(旧版整包): STANDBY\n", my_id);
                }
                return;
            }
            float new_target = int16ToFloat(target_int, scale);
            Serial.printf("[BLE调试] 设备%d(旧版)缩放后目标值: %.2f\n", my_id, new_target);
    
            if (fabs(new_target - ble_motor_target) > 0.001f) {
                ble_motor_target = new_target;
                new_command = true;
            } else {
                new_command = false;
            }
    
            char response[50];
            snprintf(response, sizeof(response), "%d:MULTI:%.2f", my_id, ble_motor_target);
            sendBLEResponse(response);
            Serial.printf("[BLE调试] 设备%d收到指令: 多电机控制(旧版整包), 目标值: %.2f\n", my_id, ble_motor_target);
            return;
        }
    
        // 其它情况：格式无效
        Serial.printf("[BLE错误] MULTI格式无效或长度不匹配，len=%d\n", data.length());
        return;
        
    } else if (packet_type == PACKET_TYPE_MULTI_STRUCT) {  // 结构体多电机控制包
        Serial.printf("[BLE调试] 开始处理结构体多电机包，长度: %d\n", data.length());

        // 计算偏移：AA 55 03 DT COUNT | items...
        int type_offset  = has_frame_header ? 3 : 1;
        int count_offset = type_offset + 1;
        int items_offset = type_offset + 2;

        if ((int)data.length() < items_offset) {
            Serial.printf("[BLE错误] MULTI_STRUCT包长度不足，len=%d\n", data.length());
            return;
        }

        uint8_t dt = (uint8_t)data[type_offset];      // 数据类型
        uint8_t count = (uint8_t)data[count_offset];  // 条目数量

        // 根据数据类型选择缩放系数
        float scale = ANGLE_SCALE;
        if (dt == DATA_TYPE_VELOCITY) {
            scale = VELOCITY_SCALE;
            data_scale_type = 1;
            Serial.printf("[BLE调试] 使用速度缩放系数: %.2f\n", scale);
        } else if (dt == DATA_TYPE_CURRENT) {
            scale = 1000.0f;
            data_scale_type = 2;
            Serial.printf("[BLE调试] 使用电流缩放系数: %.2f\n", scale);
        } else {
            data_scale_type = 0;
            Serial.printf("[BLE调试] 使用角度缩放系数: %.2f\n", scale);
        }

        int expected_min_len = items_offset + count * 3;  // 每个条目3字节
        if ((int)data.length() < expected_min_len) {
            Serial.printf("[BLE错误] MULTI_STRUCT包长度不匹配，期望≥%d，实际: %d\n", expected_min_len, data.length());
            return;
        }

        uint8_t my_id = getMyDeviceID();
        bool found = false;  // 是否找到本设备数据的标志

        // 遍历所有条目查找本设备数据
        for (int i = 0; i < count; i++) {
            int item_offset = items_offset + i * 3;  // 每个条目3字节
            uint8_t id  = (uint8_t)data[item_offset + 0];  // 设备ID
            uint8_t vh  = (uint8_t)data[item_offset + 1];  // 数值高位
            uint8_t vl  = (uint8_t)data[item_offset + 2];  // 数值低位
            int16_t raw = (int16_t)(((uint16_t)vh << 8) | (uint16_t)vl);  // 组合为16位整数

            Serial.printf("[BLE调试] 条目%d: ID=%d 原始字节=%02X %02X 原始值=%d\n", i, id, vh, vl, raw);

            if (id == my_id) {  // 找到本设备数据
                // 模式控制：DATA_TYPE_MODE（非零=RUN，零=STANDBY）
                if (dt == DATA_TYPE_MODE) {
                    if (raw != 0) {
                        enterRun();
                        char response_mode[50];
                        snprintf(response_mode, sizeof(response_mode), "%d:MODE:RUN", my_id);
                        sendBLEResponse(response_mode);
                        Serial.printf("[BLE调试] 设备%d模式切换(STRUCT): RUN\n", my_id);
                    } else {
                        enterStandby();
                        char response_mode[50];
                        snprintf(response_mode, sizeof(response_mode), "%d:MODE:STANDBY", my_id);
                        sendBLEResponse(response_mode);
                        Serial.printf("[BLE调试] 设备%d模式切换(STRUCT): STANDBY\n", my_id);
                    }
                    // 保存到结构体用于记录
                    last_multi_struct_cmd.packet_type = PACKET_TYPE_MULTI_STRUCT;
                    last_multi_struct_cmd.device_id   = my_id;
                    last_multi_struct_cmd.data_type   = dt;
                    last_multi_struct_cmd.raw_value   = raw;
                    last_multi_struct_cmd.scaled_value= 0.0f;
                    last_multi_struct_cmd.count       = count;
                    return;
                }
                float target = int16ToFloat(raw, scale);  // 转换为浮点数

                // 保存到结构体（用于后续处理）
                last_multi_struct_cmd.packet_type = PACKET_TYPE_MULTI_STRUCT;
                last_multi_struct_cmd.device_id   = my_id;
                last_multi_struct_cmd.data_type   = dt;
                last_multi_struct_cmd.raw_value   = raw;
                last_multi_struct_cmd.scaled_value= target;
                last_multi_struct_cmd.count       = count;

                // 更新执行目标（与主循环对接）
                if (fabs(target - ble_motor_target) > 0.001f) {
                    ble_motor_target = target;
                    new_command = true;
                    Serial.printf("[BLE调试] 设备%d目标更新: %.2f\n", my_id, target);
                } else {
                    new_command = false;
                    Serial.printf("[BLE调试] 设备%d目标未改变: %.2f\n", my_id, target);
                }

                // 发送响应
                char response[50];
                snprintf(response, sizeof(response), "%d:MULTI_STRUCT:%.2f", my_id, ble_motor_target);
                sendBLEResponse(response);

                Serial.printf("[BLE调试] 设备%d收到结构体指令: DT=0x%02X, 目标=%.2f, COUNT=%d\n", my_id, dt, ble_motor_target, count);
                found = true;
                break;
            }
        }

        if (!found) {
            Serial.printf("[BLE调试] 本设备ID %d不在MULTI_STRUCT包的%d个条目中，忽略\n", my_id, count);
            return;
        }
    } else {
        // 未知包类型处理：记录并回复错误响应
        snprintf(debugMsg, sizeof(debugMsg), "未知的数据包类型: 0x%02X", packet_type);
        bleDebugPrint(debugMsg);
        
        char response[50];
        snprintf(response, sizeof(response), "%d:ERROR:UNKNOWN_PACKET", my_id);
        sendBLEResponse(response);
        
        Serial.printf("[BLE调试] 设备%d收到未知指令: 类型0x%02X\n", my_id, packet_type);
    }
}

// ============================================================================
// BLE服务器回调类
// 功能：处理BLE连接状态变化事件
// ============================================================================
class MyServerCallbacks: public BLEServerCallbacks {
    // ============================================================================
    // 函数：onConnect
    // 功能：设备连接建立时的回调函数
    // 参数：pServer - BLE服务器对象指针
    // 说明：更新连接状态标志
    // ============================================================================
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        bleDebugPrint("设备已连接");
    }

    // ============================================================================
    // 函数：onDisconnect
    // 功能：设备断开连接时的回调函数
    // 参数：pServer - BLE服务器对象指针
    // 说明：更新连接状态标志
    // ============================================================================
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        bleDebugPrint("设备已断开连接");
    }
};
// 特征值回调类
class MyCallbacks: public BLECharacteristicCallbacks {
    // ============================================================================
    // 函数：onWrite
    // 功能：接收到数据写入时的回调函数
    // 参数：pCharacteristic - 特征值对象指针
    // 说明：解析接收到的数据并调用相应的处理函数
    // ============================================================================
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();  // 获取接收到的数据
        
        if (rxValue.length() > 0) {
            char debugMsg[100];
            snprintf(debugMsg, sizeof(debugMsg), "收到数据，长度: %d", rxValue.length());
            bleDebugPrint(debugMsg);
            
            // 新增：在串口显示原始接收数据的十六进制格式
            Serial.print("[BLE接收] 原始数据(HEX): ");
            for (int i = 0; i < rxValue.length(); i++) {
                Serial.printf("%02X ", (uint8_t)rxValue[i]);
            }
            Serial.println();
            
            // 修复：添加更详细的调试信息
            Serial.printf("[BLE调试] 开始解析数据包，长度: %d\n", rxValue.length());
            
            // 修复：直接解析接收到的数据，不检查广播包头
            // 因为Python客户端发送的是直接数据，不是广播包
            parseDirectCommandData(rxValue);
        }
    }
};

// BLE服务器初始化函数
void initBLEServer() {  
    getMyDeviceID();
    
    // 统一为每台设备设置唯一ID与设备名
    my_device_id = MY_DEVICE_ID;
    char name_buf[32];
    snprintf(name_buf, sizeof(name_buf), "Motor-Controller-%d", my_device_id);

    // 初始化BLE设备，设置设备名称
    if (!BLEDevice::getInitialized()) {
        BLEDevice::init(name_buf);
    }
    
    // 创建BLE服务器
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // 创建BLE服务
    pService = pServer->createService(SERVICE_UUID);

    // 创建发送特征值（用于向PC发送数据）
    pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY  // 通知属性
                      );
    pTxCharacteristic->addDescriptor(new BLE2902());  // 添加描述符

    // 创建接收特征值（用于接收PC数据）
    pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE  // 写入属性
                      );
    pRxCharacteristic->setCallbacks(new MyCallbacks());  // 设置回调函数

    // 启动服务
    pService->start();

    // 开始广播
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);  // 添加服务UUID
    pAdvertising->setScanResponse(true);         // 设置扫描响应
    pAdvertising->setMinPreferred(0x06);          // 有助于iPhone连接
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();               // 开始广播
    
    bleDebugPrint("BLE服务器已启动，等待连接...");
    bleDebugPrint("设备名称: DFOC-Motor-Controller-0");
}


// 响应发送函数
void sendBLEResponse(const char* response) {
    if (deviceConnected && pTxCharacteristic) {
        try {
            pTxCharacteristic->setValue(response);  // 设置特征值
            pTxCharacteristic->notify();            // 发送通知
            bleDebugPrint("已发送响应");
            
            // 新增：在串口显示发送的响应内容
            Serial.printf("[BLE响应] 发送: %s\n", response);
        } catch (const std::exception& e) {
            bleDebugPrint("发送响应失败");
            Serial.printf("[BLE错误] 发送响应失败: %s\n", e.what());
        }
    } else {
        bleDebugPrint("设备未连接，无法发送响应");
    }
}

// 在主循环中需要添加连接状态管理
void BLE_Server_Loop() {
    // 处理设备连接状态变化
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);  // 给蓝牙栈时间
        if (pServer) {
            pServer->startAdvertising();  // 重新广播
            bleDebugPrint("开始广播，等待连接...");
        }
        oldDeviceConnected = deviceConnected;
    }
    
    if (deviceConnected && !oldDeviceConnected) {
        // 连接建立时的处理
        oldDeviceConnected = deviceConnected;
        bleDebugPrint("设备连接已建立");
    }
    
    // 定期发送心跳包（带设备ID，便于Python映射）
    static unsigned long lastHeartbeat = 0;
    if (deviceConnected && millis() - lastHeartbeat > 5000) {  // 每5秒发送一次
        if (pTxCharacteristic) {
            char hb[32];
            snprintf(hb, sizeof(hb), "%d:HEARTBEAT", my_device_id);
            pTxCharacteristic->setValue(hb);
            pTxCharacteristic->notify();
        }
        lastHeartbeat = millis();
    }
}
