# PMSM FOC 控制系统

基于 TI LAUNCHXL-F280049C 与 BOOSTXL-DRV8323RS 搭建的 PMSM FOC 控制项目。仓库包含实机控制源码和 MATLAB/Simulink 闭环模型，涵盖控制实现、参数配置及验证结果。

## 硬件平台

- 控制器：TI LAUNCHXL-F280049C
- 栅极驱动板：BOOSTXL-DRV8323RS
- 电机：PMSM，7 对极，4096 CPR 编码器
- 实机测试的标称直流母线：24.1 V

## 控制架构

PWM 同步触发电流采样；在电流中断中完成零偏补偿、三相电流变换和独立的 Id/Iq PI。编码器提供转子角度与速度反馈，速度 PI 产生电流参考。电压指令经过逆 Park、逆 Clarke 后，按归一化相电压线性映射并限幅为三相 PWM 占空比。应用状态机负责启动对齐、运行、受控停止、故障锁存与复位恢复。

## 主要实现

- 30 kHz 电流控制、10 ms 速度环更新；PI 使用逐次离散积分及条件积分抗饱和。
- 三相电流采样与零偏校准、eQEP 编码器反馈、ePWM 输出及 DRV8323 SPI/nFAULT 接口。
- 软件过流确认与故障状态处理；主要控制参数见 `firmware/main_final.c`，PI 实现见 `firmware/hal/pi_controller.c`。

## Hardware Validation

以下为项目实机测试结果，硬件测试与仿真结果分别记录。

| 条件 | 记录结果 |
|---|---|
| 100 rpm 下分别测试 24.1 V 至 16 V 母线电压 | 各测试点平均速度保持在约 100 rpm；这不是一次瞬态降压阶跃测试 |
| 启动 | START→RUN：30 ms；RUN→100±3 rpm 稳定：273 ms |
| 故障复位 | RESET→READY：961 ms，其中包含设计的 900 ms 对齐过程 |
| 软件故障注入 | 状态转换在约 1 ms 主循环的同一周期内被观察到；不是物理响应时间为 0 ms 的结论 |

## Simulation Validation

使用 MATLAB R2024b 对仓库中的正式模型重新运行 0–10 s 场景。模型从 t=0 起给定 100 rpm；负载在 t=3 s 从 0.0005 增至 0.001 N·m，仿真过程中未额外调整控制参数。

| 仿真指标 | 本次结果 |
|---|---:|
| 从静止首次达到 90 rpm | 301.2 ms |
| 启动峰值 / 相对 100 rpm 超调 | 100.843 rpm / 0.843% |
| 负载变化后最低速度 | 74.523 rpm |
| 负载变化后首次恢复至 ≥99 rpm | 1.194 s |
| 9–10 s 稳态速度 / q 轴电流 | 约 100 rpm / 0.02750 A |

上述数值是仿真结果，不代表实机动态性能。

## 已知边界

- 仓库未包含完整 CCS 工程配置、linker、TI device support 和 SDK。
- 仿真机械参数包含初始估计值，未宣称为实机辨识结果；firmware 与仿真使用不同的内部 q 轴符号约定，但都以正向机械转速为控制目标。
- 历史电流阶跃测试使用了不同的 Iq PI 参数，因此未列入当前版本的性能结果。

## 目录结构

```text
firmware/
  main_final.c
  clarke_park.c
  clarke_park.h
  hal/                  正式控制与硬件接口源码
simulation/
  PMSM_FOC_F280049C.slx
  motor_params.m
  control_params.m
  simulation_params.m
README.md
```
