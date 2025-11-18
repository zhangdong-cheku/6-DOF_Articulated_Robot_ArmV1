# 6-DOF_Articulated_Robot_ArmV1
六自由度关节式机械臂6-DOF Articulated Robot ArmV1
该机械臂摇操系统采用 AS5600 磁编码器和 ESP32 控制平台构建。
系统通过四路磁编码器实时采集关节角度变化，ESP32 对数据进行处理后以蓝牙广播方式发送至上位机或移动终端。
终端接收数据并将关节角映射至三维仿真环境，实现摇操系统与虚拟模型的同步交互。

This robotic arm teleoperation system is built using AS5600 magnetic encoders and the ESP32 control platform.
The system collects real-time joint angle changes through four magnetic encoder channels, and the ESP32 processes the data and broadcasts it via Bluetooth to a host device or mobile terminal.
The terminal receives the data and maps the joint angles into a 3D simulation environment, achieving synchronized interaction between the teleoperation system and the virtual model.
