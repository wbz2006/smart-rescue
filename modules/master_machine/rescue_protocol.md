# 上位机—下位机通信协议

本协议使用 USART10 与 Seasky 帧校验方式，通过不同的 `cmd_id` 区分惯导、轮里程、状态和控制命令。

## 串口参数

- USART10，PE2 为 RX、PE3 为 TX
- 115200 baud，8 data bits，no parity，1 stop bit，无硬件流控
- 帧内多字节整数和浮点数均为 little-endian；浮点数为 IEEE-754 binary32

## 帧格式

| 字段 | 长度 | 说明 |
| --- | ---: | --- |
| SOF | 1 byte | 固定 `0xA5` |
| LEN | 2 bytes | 后续数据区长度，即 `SEQ + FLAGS + PAYLOAD` 字节数 |
| CRC8 | 1 byte | 对 SOF 和 LEN（字节 0–2）计算 |
| CMD | 2 bytes | 命令 ID |
| SEQ | 1 byte | 发送端每成功发出一帧加一，按 8 bit 回绕 |
| FLAGS | 1 byte | 当前保留，发送为 `0` |
| PAYLOAD | LEN − 2 bytes | 命令数据 |
| CRC16 | 2 bytes | 对帧头至 PAYLOAD 计算，低字节先发 |

帧总长度为 `LEN + 8`。当前最大 PAYLOAD 为 128 bytes。CRC 算法沿用项目的 `crc_8`（初值 `0x00`）与 `crc_16`（初值 `0xFFFF`、多项式 `0xA001`）。

## 命令与 payload

所有字段按表格顺序紧密排列，不含 C 结构体对齐填充。时间戳为 MCU `HAL_GetTick()` 毫秒计数，按 `uint32` 回绕。

| 方向 | CMD | 名称 | 频率 | payload |
| --- | ---: | --- | ---: | --- |
| 下位机 → 上位机 | `0x0101` | IMU | 100 Hz | `timestamp_ms:uint32`, `accel[3]:float` (m/s²), `gyro[3]:float` (rad/s), `roll/pitch/yaw:float` (degree)；共 40 bytes |
| 下位机 → 上位机 | `0x0102` | 轮状态 | 20 Hz | `timestamp_ms:uint32`, `left_distance_mm:float`, `right_distance_mm:float`, `left_speed_mm_s:float`, `right_speed_mm_s:float`；共 20 bytes |
| 下位机 → 上位机 | `0x0103` | 机器人状态 | 10 Hz | `timestamp_ms:uint32`, `robot_state:uint8`, `chassis_mode:uint8`, `online_flags:uint8`, `reserved:uint8`；共 8 bytes |
| 上位机 → 下位机 | `0x0201` | 控制命令 | 上位机周期发送 | `vx_mm_s:float`, `vy_mm_s:float`, `wz_rad_s:float`, `chassis_mode:uint8`, `claw_mode:uint8`, `camera_pos:uint8`；共 15 bytes |

IMU 三轴顺序与项目 `attitude_t` 中的 XYZ 顺序相同；加速度和角速度是 BMI088/INS 当前输出值。欧拉角来自 INS 解算，单位为度。

左右轮距离是从下位机启动起累计的里程，直行前进为正，重启后归零。左右轮速度单位为 mm/s，前进为正。

控制量中 `vx` 前进为正、`vy` 向底盘右侧为正、`wz` 逆时针为正。`chassis_mode`：`0=CHASSIS_ZERO_FORCE`、`1=CAHSSIS_MOVE`；`claw_mode`：`0=CLAW_ZERO_POS`、`1=CLAW_LOCK`、`2=CLAW_RELEASE`；`camera_pos`：`0=CAMERA_ZERO_POS`、`1=CAMERA_LOAD`。下位机会拒绝长度不符、校验失败、非有限浮点数或无效模式值的控制帧。

状态帧的 `online_flags`：bit 0 表示近 200 ms 内收到有效上位机控制帧；bit 1 表示遥控器在线（仅在编译启用遥控器模式时使用）。

## 控制源与超时

`application/robot_def.h` 默认启用 `AUTO_CONTROL`。上位机控制帧超过 200 ms 未更新时，下位机将底盘切换到 `CHASSIS_ZERO_FORCE`。需要用遥控器调试时，关闭 `AUTO_CONTROL` 并启用 `REMOTE_CONTROL`。

## 帧长度示例

- IMU：`LEN=42`，完整帧 50 bytes。
- 轮状态：`LEN=22`，完整帧 30 bytes。
- 机器人状态：`LEN=10`，完整帧 18 bytes。
- 控制命令：`LEN=17`，完整帧 25 bytes。

## 实现位置

- `seasky_protocol.c/.h`：通用帧打包和校验
- `master_process.c/.h`：USART10 收发、下行控制帧解析
- `robot_task.h`：IMU 遥测
- `chassis.c`：左右轮里程和速度遥测
- `robot_cmd.c`：状态遥测和下行控制接入
