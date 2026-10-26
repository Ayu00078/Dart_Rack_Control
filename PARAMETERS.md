# 控制参数与位置速查

本文汇总当前 [控制源码](Core/Src/main.c) 中的关键参数，便于定位和修改；**实际行为以源码为准**。屏幕帧及使用顺序见 [README 的 UART7 指令](README.md#uart7-指令)。下述位置均为电机编码器**累计计数**，不是角度或毫米；`*_MS` 以毫秒计。更改运动方向、速度、行程或电流前，先核对接线、机械止挡和传动比，在受控环境逐步测试。修改文档中的数值不会更改固件，需修改 `Core/Src/main.c` 并重新构建固件。

## 同时寻零：两台 3508

按钮 `00` 在电机反馈在线且状态允许时启动两台 3508 和 M2006 同时寻零；Yaw 不参与。本组宏位于 `Core/Src/main.c` 开头的 `BASE_HOME_*` 定义区。

| 宏 | 当前值 | 作用 |
| --- | ---: | --- |
| `BASE_HOME_VELOCITY` | `-1000.0f` | 两台 3508 寻零速度目标；正负号决定代码坐标系下的方向。第二台电机安装反向，代码会反转对应电流输出。 |
| `BASE_HOME_CURRENT_LIMIT` | `3200` | 寻零速度环输出的电流**命令**限幅。 |
| `BASE_HOME_CURRENT_GATE` | `2500` | 检测碰限位使用的 CAN **反馈电流**绝对值门槛，不等同于输出限幅。 |
| `BASE_HOME_SPEED_GATE` | `50` | 同时要求解码后的电机速度绝对值不大于此值。 |
| `BASE_HOME_BLANK_MS` | `300U` | 启动后忽略碰限位判定的消隐时间。 |
| `BASE_HOME_CONFIRM_MS` | `200U` | 反馈电流及低速条件连续满足的确认时间；两电机分别计时。 |
| `BASE_HOME_TIMEOUT_MS` | `10000U` | 共同的总寻零超时；未全部完成则初始化失败。 |

判定还要求反馈距当前不超过 **20 ms**（控制函数内的字面量）。每台 3508 单独记录零点 `g_base_zero_offset[i]`；先完成者输出归零，待两台和 M2006 都寻零完成才返回 `INIT DONE`。

## M2006 寻零

以下 `TRIGGER_HOME_*` 宏也在 `Core/Src/main.c` 的宏定义区：

| 宏 | 当前值 | 作用 |
| --- | ---: | --- |
| `TRIGGER_HOME_VELOCITY` | `-5000.0f` | 寻零速度目标；反馈速度与输出电流有反向安装处理。 |
| `TRIGGER_HOME_CURRENT_LIMIT` | `6000` | 寻零时的电流命令限幅。 |
| `TRIGGER_HOME_CURRENT_GATE` | `2000` | 速度 PID **输出电流命令**绝对值的寻零门槛，不是反馈电流门槛。 |
| `TRIGGER_HOME_CONFIRM_MS` | `100U` | 命令电流达到门槛的连续确认时间。 |
| `TRIGGER_HOME_TIMEOUT_MS` | `120000U` | 寻零超时。 |

M2006 当前**不以反馈电流或低速**判断碰限位；门槛维持足够久就记录 `g_trigger_zero_offset`。调小门槛有误判风险，必须现场核验实际限位动作。

## 四档目标位置：代码内数组（不是宏）

到 `Core/Src/main.c` 的 `launcher_apply_commands()` 查找 `trigger_positions[4]` 和 `load_positions[4]`。两数组相同索引对应同一屏幕按钮：选档时先让 M2006 移至目标；`01` 在初始化、选档及 M2006 到位后才让两台 3508 装载。3508 各自以自身寻零点计算目标位置。

| 串口屏按钮字节 | 数组索引 | M2006 `trigger_positions` | 两台 3508 `load_positions` |
| --- | ---: | ---: | ---: |
| `0x10` | 0 | `36500000L` | `760000L` |
| `0x11` | 1 | `24500000L` | `920000L` |
| `0x12` | 2 | `12500000L` | `1020000L` |
| `0x13` | 3 | `500000L` | `1060000L` |

修改某一档，核对同一索引下两个数组的值。`LOAD_POSITION_MIN` = `-100000L`、`LOAD_POSITION_MAX` = `1070000L` 虽然已定义，**当前屏幕四档路径未使用这些宏来限制目标**；只改它们不能限制四档行程。

## 自动装载、回程和发射

| 宏 | 当前值 | 作用 |
| --- | ---: | --- |
| `LOAD_AUTO_LOCK_MS` | `3000U` | `01` 启动装载后按时间切到锁紧，**不等待** 3508 到位。 |
| `SERVO_LOCK_WAIT_MS` | `1000U` | 舵机锁紧后等待，再启动两台 3508 回程。 |
| `LOAD_RETURN_POSITION` | `5000L` | 屏幕自动流程的 3508 回程目标；相对各自的寻零点。 |
| `FIRE_SERVO_DELAY_MS` | `1000U` | 接受发射后先停 3508 输出，等待后再让舵机释放。 |
| `FIRE_TRIGGER_TIME_MS` | `3000U` | 舵机释放后的保持时间，到时舵机复位。 |

`LAUNCHER_LOAD_DEFAULT` = `367878L`、`LAUNCHER_HOME_DEFAULT` = `55000L` 是初始化时的默认目标，**不是**四档装载目标及屏幕自动回程目标；屏幕自动回程使用 `LOAD_RETURN_POSITION`。发射在寻零中被拒绝，但目前 `FAULT` 状态仍可请求发射，不能将故障视为安全闭锁。

## 到位判定、反馈和增流

| 宏 | 当前值 | 作用 |
| --- | ---: | --- |
| `MOTOR_FEEDBACK_TIMEOUT_MS` | `500U` | 最近一次 CAN 反馈超过此时间则视为离线。 |
| `TRIGGER_TARGET_TOLERANCE` | `10000L` | M2006 目标位置误差容差。 |
| `TRIGGER_VELOCITY_TOLERANCE` | `100.0f` | M2006 到位时的速度绝对值上限。 |
| `LOAD_TARGET_TOLERANCE` | `5000L` | 两台 3508 到位时各自的位置误差容差。 |
| `TARGET_VELOCITY_TOLERANCE` | `30.0f` | 两台 3508 到位时的滤波速度绝对值上限。 |
| `POSITION_STABLE_TIME_MS` | `100U` | 满足到位与低速条件后连续稳定的确认时间。 |
| `MOTION_TIMEOUT_MS` | `10000U` | 非自动装载/回程位置命令的超时。 |
| `TRIGGER_MOTION_TIMEOUT_MS` | `120000U` | M2006 目标位置运动超时。 |
| `YAW_TARGET_TOLERANCE` / `YAW_VELOCITY_TOLERANCE` | `1000L` / `120.0f` | Yaw 位置/速度到位容差。 |
| `YAW_MOTION_TIMEOUT_MS` | `10000U` | Yaw 目标运动超时。 |

装载中的额外卡滞增流**不是寻零判定**：`LOAD_STALL_ERROR_THRESHOLD` = `50000L`（位置误差），`LOAD_STALL_VELOCITY_THRESHOLD` = `100.0f`（速度），`LOAD_STALL_DETECT_MS` = `200U`（持续检测），`LOAD_STALL_BOOST_MS` = `500U`（单次增流时长），`LOAD_STALL_MAX_ATTEMPTS` = `3U`（每电机最多次数），`LOAD_STALL_BOOST_CURRENT` = `16384`（增流电流命令）。提高它会增加机械负载。

## 舵机、Yaw 与正常运动速度

| 宏 | 当前值 | 作用 |
| --- | ---: | --- |
| `SERVO_LOCK_0` / `SERVO_LOCK_1` | `55U` / `0U` | 两路舵机的锁紧角度。 |
| `SERVO_AVOID_0` / `SERVO_AVOID_1` | `95U` / `35U` | 两路舵机的让位/释放角度。 |
| `SERVO_MIN_PULSE` / `SERVO_MAX_PULSE` | `500U` / `2500U` | 舵机角度换算的 PWM 脉宽计数；计数器为 1 MHz。 |
| `SERVO_MAX_ANGLE` | `180U` | 舵机角度输入上限。 |
| `YAW_ANGLE_DEFAULT` | `40000L` | Yaw 初始目标累计计数；本次初始化不为 Yaw 寻零。 |
| `YAW_ANGLE_MIN` / `YAW_ANGLE_MAX` | `10000L` / `110000L` | 已定义的范围宏；调整前检查实际调用路径，不要假设屏幕四档受其限制。 |
| `MOTOR_CONTROL_PERIOD_MS` | `1U` | 主循环尝试运行电机控制的周期。 |

正常位置运动的速度上限**不是单独的宏**：在 `Core/Src/main.c` 的 `launcher_init()` 中查找 `pid_config(&g_load_angle_pid[0/1], ...)`（两台 3508 的最后一个参数均为 `4000.0f`）、`pid_config(&g_trigger_angle_pid, ...)`（M2006 为 `8000.0f`）和 `pid_config(&g_yaw_angle_pid, ...)`（Yaw 为 `300.0f`）。位置环输出目标速度；速度环再输出受限的 CAN 电流命令。修改寻零速度应改前述 `*_HOME_VELOCITY`，不要仅修改正常位置环。

> 本文仅记录代码当前配置，不表示 Keil 目标编译、烧录或硬件验证通过；寻零方向、电流反馈、行程及故障发射分支均须受控验证。