# Changelog

本项目版本变更记录。格式遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，版本号采用语义化版本（SemVer）。

## [v2.02] - 2026-10-31

首次纳入 GitHub 版本管理，记录当前固件功能基线。

### 新增
- 差速驱动小车固件（Renesas RA6M4，FSP / RASC 生成，CMake 构建，含 Debug / Release Presets）。
- 双轮独立 PID 速度闭环（默认 Kp=0.09 / Ki=0.01 / Kd=0，20ms 周期），带抗积分饱和、速度一阶低通滤波、目标速度斜坡、死区前馈（55%）与平滑启动逻辑。
- 霍尔编码器测速（13PPR × 4 × 22.569 ≈ 1174 计数/转，差分换算为每 500ms 速度）。
- 电机 PWM 驱动（GPT1 左轮 / GPT2 右轮，20kHz，单象限输出，含方向处理）。
- CAN 协议 v0.2（500kbps 经典帧）：0x180 速度/模式命令、0x181 参数读写、0x280 里程与状态上报（100Hz）、0x282 参数应答、0x300 电压/温度/故障上报（10Hz）；含 IDLE / TELEOP / AUTO / FAULT 状态机、200ms 指令超时停车、bus-off 等总线故障检测。
- 串口命令解析（SCI9 有线调试 + SCI0 无线 MX-02P 蓝牙，共用命令集）：`L:/R:` 独立轮速、`T:` 原地旋转、`Kp:/Ki:/Kd:` 在线调参。
- 内置 micro-ROS 依赖目录（`microros/`）。

### 工程
- 仓库纳入 git 管理，忽略编译缓存（`build/`、`.cache/`）。

### 说明
- `0x300` 中的电压（12.0V）与温度（25°C）为固定占位值，未接 ADC / 测温。
- 硬件 TELEOP/AUTO 模式开关校验默认关闭（小车无此物理开关）。
