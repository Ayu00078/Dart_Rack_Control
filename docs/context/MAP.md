# 工程映射

以下仅是 2026-10-26 对仓库内 README、目录和配置入口的核对；不是工具安装或硬件部署验证。

| 对象 | 位置及关系 |
| --- | --- |
| 控制逻辑及外设源码 | [Core/Src/](../../Core/Src/)，公共头文件在 [Core/Inc/](../../Core/Inc/)；指令与部分参数在 [main.c](../../Core/Src/main.c) |
| STM32CubeMX 外设配置 | [Dart_Rack_Control_v1t.ioc](../../Dart_Rack_Control_v1t.ioc)，记录 STM32F427I(G-I)Hx 和 CAN1 配置 |
| Keil 工程 | [MDK-ARM/Dart_Rack_Control_v1t.uvprojx](../../MDK-ARM/Dart_Rack_Control_v1t.uvprojx)，目标 STM32F427IIHx，配置 Keil.STM32F4xx_DFP.3.1.1 和 ARMCC 5；工具实际安装待核实 |
| 库与启动文件 | [Drivers/](../../Drivers/) 含 CMSIS / STM32F4 HAL；启动文件在 [MDK-ARM/startup_stm32f427xx.s](../../MDK-ARM/startup_stm32f427xx.s) |
| 配置的构建输出 | 工程 OutputDirectory 为 `MDK-ARM/Dart_Rack_Control_v1t/`，配置生成 hex；该目录及常见产物见 [.gitignore](../../.gitignore)，不代表本次构建成功 |

部署设备、串口接线与本地工具位置：待核实；不记录个人设备私有路径。映射变动时修订本页。