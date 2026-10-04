# AGENTS.md — 自平衡小车项目技术参考文档（26915 FreeRTOS 版）

> 本文档供 AI 辅助开发时参考。最后更新：2026-09-15

---

## 1. 项目概述

基于 STM32F103C8T6 的自平衡小车项目，使用 STM32 HAL 库开发，**FreeRTOS 实时操作系统** 进行多任务调度。通过 ICM42688P 六轴 IMU 获取姿态，配合编码器测速，实现直立、速度、转向三环 PID 控制。

**开发工具**: CLion + arm-none-eabi-gcc + CMake + STM32CubeMX

**参考项目**: `26914_balance`（裸机版本，含完整驱动代码）

**当前状态：项目初始化阶段，待 CubeMX 生成工程并搭建 FreeRTOS 任务架构。**

---

## 2. 硬件配置

### 2.1 MCU

| 参数 | 值 |
|------|-----|
| 型号 | STM32F103C8T6 (LQFP48) |
| 内核 | ARM Cortex-M3 |
| 主频 | 72 MHz (HSE 8MHz, PLL ×9) |
| Flash | 64 KB |
| RAM | 20 KB |

### 2.2 外设连接

| 外设 | 接口 | 功能 | 引脚 |
|------|------|------|------|
| ICM42688P | SPI1 | 6轴IMU (加速度+陀螺仪) | PA4(CS), PA5(SCK), PA6(MISO), PA7(MOSI) |
| SSD1306 OLED | I2C1 | 128×64 显示屏 | PB8(SCL), PB9(SDA) |
| AT24CXX EEPROM | I2C1 | 参数存储 | PB8(SCL), PB9(SDA) (共享I2C总线) |
| DRV4950 电机驱动 | TIM2 PWM | 双H桥直流电机驱动 | PA15(CH1), PB3(CH2), PB10(CH3), PB11(CH4) |
| 左电机编码器 | TIM3 | 硬件编码器接口 | PB4(CH1), PB5(CH2) |
| 右电机编码器 | TIM4 | 硬件编码器接口 | PB6(CH1), PB7(CH2) |
| EC11 旋转编码器 | GPIO | 按键+旋转 | PB0(B相), PB1(A相) |
| 用户按键 | GPIO | KEY (上拉) | PC13 |
| IMU SPI片选 | GPIO | CS (低有效) | PA4 |
| USART1 | UART | 调试串口 (DMA收发) | PA9(TX), PA10(RX) |
| USART2 | UART | 备用串口 | PA2(TX), PA3(RX) |
| ADC1 | ADC | 2通道 (电池电压?) | PA0(IN0), PA1(IN1) |

---

## 3. CubeMX 配置要点

- **目标工具链**: STM32CubeIDE（生成 CMake 工程）
- **时钟源**: HSE (8MHz) → PLL ×9 → SYSCLK = 72MHz
- **APB1**: 36MHz, **APB2**: 72MHz
- **NVIC 优先级分组**: GROUP_4 (4 bits preemption, 0 bits sub)
- **FreeRTOS**: CMSIS_V2 接口

### 不使用的外设

| 外设 | 原因 |
|------|------|
| USB_DEVICE | 不需要 USB CDC，使用 USART1 调试 |
| TIM1 CH1 | 未使用，保留备用 |

---

## 4. FreeRTOS 任务架构（规划）

| 任务 | 优先级 | 周期 | 功能 |
|------|--------|------|------|
| IMU_Task | 高 (5) | 1ms | 读取IMU数据 → Mahony姿态解算 |
| Control_Task | 高 (5) | 5ms | PID控制回路（直立环/速度环/转向环） |
| Encoder_Task | 中 (3) | 10ms | 编码器读取 → 速度计算 |
| Display_Task | 低 (1) | 50ms | OLED刷新显示 |
| KeyScan_Task | 低 (1) | 20ms | 按键扫描 → 功能切换 |

### 任务间通信

- **消息队列**: IMU 数据（欧拉角）从 IMU_Task 传递到 Control_Task
- **全局变量**: 编码器速度（需互斥锁保护）
- **事件标志**: 校准请求、模式切换

---

## 5. 软件模块（从参考项目复用）

### 5.1 ICM42688 驱动 (`Core/Src/ICM42688.c`)

- `ICM42688_Init(himu, config)` — 初始化，检查 WHO_AM_I (0x47)
- `ICM42688_ReadSensorData(himu)` — 读取6轴数据，转换为 g 和 dps
- 配置: ±8g / ±500dps / 1kHz ODR

### 5.2 MahonyAHRS 姿态解算 (`Core/Src/MahonyAHRS.c`)

- `MahonyAHRS_Init(mf, sample_freq, kp, ki)` — 初始化 (Kp=2.0, Ki=0.05)
- `MahonyAHRS_Update(mf, gyro, accel, dt)` — 更新姿态
- `MahonyAHRS_GetEulerAngles(mf, angles)` — 获取欧拉角
- `IMU_Calibrate(himu, calib, samples)` — 静态校准

### 5.3 DRV4950 电机驱动 (`Core/Src/DRV4950.c`)

- `MotorInit(hmotor, hpwm, ChannelIN1, ChannelIN2)` — 初始化
- `MotorDutySet(hmotor, FloatDuty, BrakeState)` — 设置占空比 (-1.0~+1.0)
- `MotorState(hmotor, WorkState)` — 启停控制

### 5.4 PID 控制器 (`Core/Src/pid.c`)

- `PID_Init(PID, Kp, Ki, Kd, T, MAX, MIN, I_threshold, E_stable)` — 初始化
- `PID(PID, SET, Actual)` — 计算PID输出
- **注意**: 此为参考实现，后续需自己重写并接入 FreeRTOS 任务

### 5.5 EC11 编码器 (`Core/Src/ec11.c`)

- `KeyScan(GPIO_Port, Pin)` — 按键扫描
- `EncoderScan(hec11)` — 旋转方向检测

### 5.6 OLED 显示 (`Core/Src/oled.c`)

- `OLED_Init()` — 初始化
- `OLED_PrintString(x, y, str, font, color)` — 字符串绘制
- `OLED_ShowFrame()` — 刷新显示

### 5.7 AT24CXX EEPROM (`Core/Src/24cxx.c`)

- `EP24C_WriteFloat/ReadFloat` — 浮点数读写
- 用于存储 PID 参数（后续实现）

---

## 6. 引脚定义 (`Core/Inc/main.h`)

```c
#define KEY_Pin         GPIO_PIN_13
#define KEY_GPIO_Port   GPIOC

#define CS_Pin          GPIO_PIN_4
#define CS_GPIO_Port    GPIOA

#define B_Pin           GPIO_PIN_0     // EC11 B相
#define B_GPIO_Port     GPIOB

#define A_Pin           GPIO_PIN_1     // EC11 A相
#define A_GPIO_Port     GPIOB
```

---

## 7. 编译配置

- **工具链文件**: `cmake/gcc-arm-none-eabi.cmake`
- **MCU 标志**: `-mcpu=cortex-m3` (无 FPU)
- **链接脚本**: `STM32F103C8Tx_FLASH.ld` (64KB Flash / 20KB RAM)
- **构建预设**: Debug (-O0 -g3) / Release (-Os)
- **烧录**: OpenOCD + ST-Link

---

## 8. 开发路线图

| 阶段 | 内容 | 状态 |
|------|------|------|
| 1 | 搭建 CLion + arm-none-eabi-gcc 编译环境 | 待开始 |
| 2 | CubeMX 生成工程 (STM32CubeIDE 工具链) | 待开始 |
| 3 | 复制驱动文件，编译验证 | 待开始 |
| 4 | 引入 FreeRTOS，创建任务架构 | 待开始 |
| 5 | 实现直立环 PID | 待开始 |
| 6 | 实现速度环 PID | 待开始 |
| 7 | 实现转向环 PID | 待开始 |
| 8 | OLED 调参界面 | 待开始 |
| 9 | EEPROM 参数存储 | 待开始 |
| 10 | ADC 电池检测 | 待开始 |

---

## 9. 与旧项目（26914）的对比

| 项目 | 26914_balance | 26915_self_balance_car |
|------|---------------|------------------------|
| 架构 | 裸机前后台 | FreeRTOS 多任务 |
| 调度 | TIM2 中断分频 | FreeRTOS 任务切换 |
| 通信 | USB CDC | USART1 串口 |
| IDE | Keil MDK-ARM | CLion + CMake |
| 编译器 | ARMCC V5 | arm-none-eabi-gcc |
| PID 实现 | 已写但未接入 | 需自己实现 |

---

## 10. 注意事项

1. **启动文件**: 需要 GNU AS 格式的 `startup_stm32f103xb.s`（非 Keil ARMCC 格式）
2. **链接脚本**: 需要 `.ld` 文件定义 Flash/RAM 区间
3. **FreeRTOS 堆**: 默认 Heap_4 方案，需根据任务数量调整 `configTOTAL_HEAP_SIZE`
4. **中断优先级**: FreeRTOS 需要 `configLIBRARY_LOWEST_INTERRUPT_PRIORITY` 和 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 正确配置
5. **编码器**: TIM3/TIM4 使用中心对齐计数 (CNT 初始值 0x7FFF)
