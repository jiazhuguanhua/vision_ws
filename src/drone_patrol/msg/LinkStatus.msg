# 通信链路状态消息
Header header
bool ble_connected       # BLE连接状态
bool uart_connected      # UART连接状态
float32 signal_strength  # 信号强度 (dBm)
int32 packet_loss        # 丢包率 (百分比)
string last_command      # 最后接收的命令
string device_status     # 设备状态信息
float32 battery_level    # 电池电量百分比
