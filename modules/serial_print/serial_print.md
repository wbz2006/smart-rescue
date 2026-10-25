# UART7 串口打印

差速底盘运行后，`ChassisTask()` 每隔至少 20 ms 输出一行 CSV，约 50 Hz。
VOFA+ 选择 **FireWater** 协议；普通串口助手也可直接查看文本。

## 连接和设置

- 开发板 UART7_TX（PE8）接 USB 转串口模块的 RX，双方 GND 共地，使用 3.3 V TTL 电平。
- 仅查看打印时无需连接开发板 RX（PE7）。DAPLink 的 SWD 连接本身不传输 UART7 数据。
- 选择 USB 转串口模块对应的 COM 端口，设置为 **921600、8 数据位、无校验、1 停止位、无流控**。
- 下载新固件并运行，使用 Cortex-Debug 时需在停到 `main` 后点击“继续”。

## 通道顺序

| 通道 | 内容 | 单位 |
| --- | --- | --- |
| 1 | 左轮速度 PID 目标 | 度/秒 |
| 2 | 左轮速度 PID 反馈 | 度/秒 |
| 3 | 左轮速度 PID 误差 | 度/秒 |
| 4 | 右轮速度 PID 目标 | 度/秒 |
| 5 | 右轮速度 PID 反馈 | 度/秒 |
| 6 | 右轮速度 PID 误差 | 度/秒 |

目标值来自 `speed_PID.Ref`，已包含电机方向设置；反馈和误差分别来自同一速度 PID 的 `Measure` 和 `Err`。
输出为纯数值，例如 `120.0,118.5,1.5,120.0,119.0,1.0`，以 `\r\n` 结尾。
电机静止或遥控器离线时仍会周期输出，数值可能保持不变。

## 在任务中打印其他数据

包含 `serial_print.h`，在 FreeRTOS 任务中调用：

```c
SerialPrintf("tick=%lu, value=%.1f\r\n", (unsigned long)HAL_GetTick(), value);
```

`BSPInit()` 已调用 `SerialPrintInit()`；UART7 直接使用 HAL 阻塞发送，不注册 DMA 接收。
Makefile 已启用 newlib-nano 的 `_printf_float`，支持 `%f`。
接口使用互斥量，仅用于调度器启动后的任务上下文，勿在中断或关闭中断的初始化阶段发送。
使用 FireWater 绘图时，请保持同一串口每行都是相同数量的数值，不混入带名称的日志。
