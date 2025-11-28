烧录前准备
在arduino_project中
1.修改int Motor_PP = 7;，将极对数修改为自己使用电机的极对数。
2.setPowerSupplyVoltage(15.6);  //!< 设置供电电压：15.6V，可设置为自己电源的供电电压，建议不超过15.6V

在Ble_Handler.cpp和Ble_Handler.h中
1.可通过修改uint8_t my_device_id = 6;  #define MY_DEVICE_ID 6  ,设置自己设备的ID号

在FOC.h中
1.#define GEAR_RATIO 225.0f，用来设置关节的减速比