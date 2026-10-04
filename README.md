# Self Balance Car 26915

基于 STM32F103C8T6 的自平衡小车项目，使用 **FreeRTOS** 实时操作系统进行多任务调度，通过 IMU 姿态解算和 PID 控制实现平衡。

## 架构

- **调度方式**: FreeRTOS 多任务
- **通信方式**: USART1 串口调试
- **开发工具**: CLion + arm-none-eabi-gcc + CMake + STM32CubeMX

## 硬件

| 组件 | 型号 | 接口 |
|------|------|------|
| MCU | STM32F103C8T6 | - |
| IMU | ICM42688P (6轴) | SPI1 |
| 电机驱动 | DRV4950 (双H桥) | TIM2 PWM |
| 显示屏 | SSD1306 OLED 128×64 | I2C1 |
| EEPROM | AT24C256 (32KB) | I2C1 |
| 编码器 | EC11 + 电机霍尔编码器 | GPIO / TIM3 TIM4 |

## 项目结构

```
├── Core/
│   ├── Inc/              # 头文件
│   └── Src/
│       ├── main.c              # 主程序入口
│       ├── app/                # 应用层
│       │   ├── pid_controller.c/h
│       │   ├── balance_control.c/h
│       │   └── state_machine.c/h
│       ├── bsp/                # 板级驱动
│       │   ├── ICM42688.c/h
│       │   ├── MahonyAHRS.c/h
│       │   ├── DRV4950.c/h
│       │   ├── ec11.c/h
│       │   ├── oled.c/h + font.c/h
│       │   └── 24cxx.c/h
│       └── ...                 # CubeMX 生成的 HAL 初始化文件
├── Drivers/              # HAL 库 + CMSIS
├── cmake/
│   ├── gcc-arm-none-eabi.cmake
│   └── stm32cubemx/
├── FreeRTOS/             # FreeRTOS 内核
├── CMakeLists.txt
├── CMakePresets.json
├── STM32F103C8Tx_FLASH.ld
├── openocd.cfg
├── AGENTS.md             
└── README.md
```

## 编译方式

### CLion + CMake

1. 打开 CLion → 文件 → 打开 → 选择本项目文件夹
2. CLion 自动检测 CMakeLists.txt
3. 选择构建预设：Debug 或 Release
4. 构建 → 编译项目

### 烧录

```bash
# 命令行烧录
openocd -f openocd.cfg -c "program build/Debug/26915_self_balance_car.elf verify reset exit"
```

## 开发状态

项目初始化阶段，驱动文件待从参考项目复制。详见 [AGENTS.md](AGENTS.md)。

## 参考项目

- `26914_balance` — 裸机版本，含完整驱动代码

## License

MIT
