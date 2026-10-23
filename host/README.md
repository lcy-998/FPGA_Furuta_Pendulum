# Furuta 摆串口上位机

## 启动

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r host\requirements.txt
.\.venv\Scripts\python.exe host\app.py
```

选择逻辑派 FPGA 的 USB-Serial COM 口后连接。串口参数为 115200 baud、8N1。

## 数据与协议

所有帧均为 `A5 | type | seq | length | payload | CRC16-CCITT-FALSE低字节 | CRC高字节`。CRC 初值 `FFFF`，覆盖 `type` 到 payload。多字节整数均小端、有符号。

| 类型 | 方向 | 载荷 |
| --- | --- | --- |
| `01` | FPGA → PC | 每 10 ms 一帧，8 个 int32 Q16.16：`theta, dtheta, alpha, dalpha, theta_ref, dtheta_ref, alpha_ref, dalpha_ref`。角度单位 rad，角速度单位 rad/s。 |
| `10` | PC → FPGA | 5 个 int32 Q10：`K1..K5`。FPGA 检查每个数可由有符号 18 位表示，并在采样边界同时应用。 |
| `81` | FPGA → PC | 1 字节状态：`00` 已应用，`01` 增益范围错误；seq 回显命令序号。 |

摆杆的角度/角速度目标当前为 0；转臂目标与控制器轨迹发生器输出相同。串口接收保持 100 Hz，界面以 20 Hz 合并刷新曲线和动画。曲线保留最近 2000 个点，CSV 环形缓冲区最多保留 60000 条记录（约 10 分钟）。

## 硬件接线

工程约束将 FPGA TX/RX 连接到 F12/F13 板载 USB-Serial 信号；板载 USB 连接 PC 即可。若使用外接 USB-TTL，需交叉连接 TX/RX 并共地，且使用 3.3 V UART 电平。传感器 ADC 串口仍在 K11。
