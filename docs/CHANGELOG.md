# Changelog

本项目所有值得记录的变更都记录在此文件中。

格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，
版本号遵循 [语义化版本 Semantic Versioning](https://semver.org/lang/zh-CN/)。

<!-- 版本分类：Added 新增 / Changed 变更 / Fixed 修复 / Removed 移除 -->

## [0.4.0] - 2026-10-16

交互与任务编排优化版本：新增画图 / 直线任务的「暂停 / 继续」与串口速度调节，按键一由循环执行改为「按一次播放一次」，动作组 A / B / C 重构为两段式衔接，示教的 z 补偿改为以首点为基准的逐点独立补偿。程序由 1211 行扩展到 1298 行。

### Added

- **画图 / 直线任务「暂停 / 继续」** `yaogan/yaogan.ino`
  - 新增状态变量 `linePaused` / `linePauseStart` 与判定函数 `drawTaskBusy()`（直线插值中或直线串联 `Line1` 处于 1–4 段视为任务进行中）；
  - 新增 `toggleLinePause()`：暂停时冻结直线推进（`moveLineTick()`）与串联计时（`Action_Linenext()`），舵机停在原处；恢复时重置步进计时基准 `lineLastTime` 并补偿串联等待基准 `nowtime`，从原处继续而非重头开始；
  - 暂停期间忽略直线推进，示教暂停逻辑不受影响。
- **串口命令扩展** `parseAndControl()`
  - `4`：等同按下按键一（画图任务进行中 = 暂停 / 继续；空闲 = 播放动作 A → B → C）；
  - `5`：画图 / 直线任务暂停 / 继续；
  - `H` / `L`：调节舵机整体速度，`TURN_INTERVAL ±5ms`。
- **按键统一处理函数** `onButton1()`：按键一与串口 `4` 共用同一入口，按「任务进行中暂停 / 空闲播放动作」分流。

### Changed

- **取消「循环模式」**：移除 `repeatMode` 变量与主循环中的自动循环代码块；按键一由「循环触发夹取」改为「**每按一次播放一次**」，依次 A → B → C，不再自动连续循环。
- **动作组 A / B / C 重构为两段式**：动作组主体改为「动作队列 + `Line1`（5 / 6 / 7）串联标记」，由 `Action_Linenext()` 在队列走完并延时 1s 后继续执行后续轨迹，使抓取动作两段衔接更平滑。
- **示教 z 补偿策略重构**：删除全局 `Z_COMP_MM` 及 `moveLineTo()` 内按起终点径向距离的自适应补偿；改为示教专用 `TEACH_Z_COMP_MM` / `TEACH_R_DEADBAND`——以第 1 个点为基准，逐点比较其径向距离 r（比首点更近 +1mm、更远 -1mm、等距不补），各点独立补偿、偏差最多 ±1mm 不累积；`teachTick()` 播放时改用各点自身 z 而非统一首点 z。
- **夹爪角度与行程重新标定**：`OPENCLAW` 60→90、`CLOSECLAW` 130→180、`Cpos` 初始 60→90；Claw 行程 `MAXCpos` / `MINCpos` 由 130 / 60 → 180 / 90。
- **按键四回中**：额外清除画图暂停状态（`linePaused=false`）并取消在途直线串联（`Line1=0`）。
- 清理未使用的 `speed` 变量与部分冗余注释；`Line1==3` 分段终点由 `moveLineTo(90,0,51)` 修正为 `(90,0,50)`。

### Removed

- 移除全局摩擦力补偿宏 `Z_COMP_MM` / `Z_MIN_MM` / `Z_MAX_MM`（由示教专用补偿 `TEACH_Z_COMP_MM` 取代）；
- 移除 `repeatMode` 循环模式变量与对应主循环代码块。

## [0.3.0] - 2026-10-12

运动学解算与轨迹控制版本：机械臂控制方式由「直接给定舵机角度」升级为「给定笛卡尔坐标 (x, y, z)」，新增正 / 逆运动学解算、直线插值运动、动作帧录制 / 回放与五点示教功能；同时重标定舵机行程并提升移动速度。程序由 `yaoganmearm/yaoganmearm.ino` 迁移重命名为 `yaogan/yaogan.ino`（331 → 1211 行）。

### Added

- **逆运动学解算** `yaogan/yaogan.ino`
  - `solveIK()`：由爪尖坐标 (x, y, z) 解算底座回转角、肩关节角、肘关节角，采用余弦定理与 `atan2` / `acos` 的解析解（定义 φ 从 +y 轴量起），含可达性校验（目标过近 / 超出 L1+L2 臂展返回无解）；
  - `computeJointAngles()`：关节角 → 舵机 PWM 值的统一换算（零点标定 `MIDDLE_ZERO` / `LEFT_ZERO` / `RIGHT_ZERO` + 方向系数 `MIDDLE_DIR` / `RIGHT_DIR` / `LEFT_DIR`），并做三舵机行程校验，越界时以 `limitServo` 标记具体舵机；
  - `driveIK()` / `driveDirect()`（`G` 命令）：坐标直接逆解驱动的调试 / 验证通道。
- **正运动学解算** `solveFK()`：由当前舵机值 (Mpos / Rpos / Lpos) 反推爪尖坐标；新增串口 `0` 命令实时查询当前位置 `reportCurrentPosition()`。
- **直线插值运动** `moveLineTo()` / `moveLineTick()`
  - 起点、终点各逆解一次，对 M / R / L 三个舵机角做**误差累积等比例插值**（每步累加增量、够 ±1° 才转 1°），实现近似直线轨迹，取代早期「逐点推进」方案；
  - 步数由变化量最大的舵机增量决定；速度由 `LINE_INTERVAL_MS`（普通）/ `TEACH_LINE_INTERVAL_MS`（示教，更慢）控制；
  - 新增 **摩擦力 / 自重误差自适应补偿**（`Z_COMP_MM`）：按目标点相对当前点的径向距离「近→远压低、远→近抬高」z，改善画线精度，并做 `Z_MIN_MM` / `Z_MAX_MM` 上下限保护；
  - 动作组扩展：`Action_Line()` / `Action_Line2()` / `Action_V()` 及串联推进 `Action_Linenext()`。
- **动作帧录制 / 回放** `Frame()` / `Reproduce()`
  - 30ms（`REC_SAMPLE_MS`）周期采样 + 阈值抽帧：爪尖位移 > `REC_DIST_MM`(10mm) 或爪子角度变化 > `REC_CLAW_DEG`(40°) 才记一帧，最多 `MAX_FRAMES`(50) 帧，存入二维数组；
  - 回放逐帧入队（队列有空位才送一帧），避免超出队列容量丢帧，播放一遍后自动停止；
  - 按键 2 录制、按键 3 播放，播放中按键 2 暂停 / 继续。
- **五点示教模式** `teachRecordPoint()` / `teachPlayStart()` / `teachTick()`
  - 串口 `8` 进入示教模式、`7` 返回普通帧录制模式；
  - 按键 3 记录当前爪尖坐标点（z 锁定为首点高度，保证所有点在同一水平面），最多 `TEACH_MAX_POINTS`(5) 点；
  - 播放流程：先回初始位 → 非阻塞等待 `TEACH_HOME_WAIT_MS`(2s) → 逐点 `moveLineTo` 走直线 → 每点到达后停留 `TEACH_SETTLE_MS`(1s) 再走下一段（全程非阻塞）；示教录制 / 播放中爪子保持闭合夹笔。
- **串口命令扩展** `parseAndControl()`：新增 `0` 查询坐标、`G` 直接逆解、`7` / `8` 模式切换、`x,y,z` 舵机角度设置。

### Changed

- **程序目录与文件重命名**：`yaoganmearm/yaoganmearm.ino` → `yaogan/yaogan.ino`。
- **按键事件检测重构**：`ButtonState` 结构体新增 `pressed` 字段，`updateButton()` 由位掩码返回改为逐键 `pressed` 事件，`loop()` 中按事件分发；按键 4 回中时一并清理直线插值、示教播放等在途状态。
- **舵机行程与初始位置重新标定**：`OPENCLAW` 120→60、`CLOSECLAW` 180→130、`Cpos` 初始 135→60；行程范围 `MINRpos` 45→20、`MAXLpos` 120→130、`MINLpos` 20→10、`MAXCpos` 180→130、`MINCpos` 120→60。
- **舵机节拍提速**：`TURN_INTERVAL` 30→15ms（舵机移动速度约提升一倍）。

### Fixed

- **画线只能画出圆弧且抖动**：舍弃早期逐步推进方案，改用「逆解 + 误差累积等比例插值」，直线轨迹更平滑、速度可控；
- **`xyz` 空参数指令导致舵机顶到极限位**：新增 `x` / `y` / `z` 后必须紧跟数字或负号的格式校验；
- **直线 / 示教与摇杆、动作队列争抢舵机**：摇杆在动作队列非空或直线插值期间暂停输入，各推进函数统一做互斥。

## [0.2.0] - 2026-09-30

非阻塞舵机控制重构与按键遥控功能版本：消除 `delay()` 阻塞隐患，新增动作队列状态机，实现按键循环夹取与随时回中。

### Changed

- **舵机控制非阻塞化重构** `yaoganmearm/yaoganmearm.ino`
  - `turn()` 由原先依赖 `delay()` 的逐度阻塞移动，重构为基于 `millis()` 的非阻塞节拍控制：未到 `TURN_INTERVAL` 时立即返回、由主循环持续推进，彻底消除动作执行期间对主循环的阻塞；
  - 新增动作队列机制：`ServoTarget actionQueue[]` 环形队列 + `actionPhase` 当前舵机阶段 + `Action()` 入队 + `actionTick()` 逐节拍推进，实现多组动作的有序排队执行；
  - A / B / C 动作组由逐条 `turn()` + `delay()` 改写为 `Action()` 队列式编排；
  - 动作执行期间暂停摇杆输入（`joystickControl()` 内 `queueCount>0` 时直接返回），避免与动作争抢同一舵机。

### Added

- **多按键掩码式检测** `updateButton()`：由单一按键返回升级为一次性扫描 4 路按键并以位掩码（`KEY1_MASK`–`KEY4_MASK`）返回，支持同一时刻多按键状态的组合判定。
- **按键 1 循环夹取逻辑**：按 `A → B → C` 顺序循环切换动作组（`Action_Mode` / `repeatMode` 状态机），在队列空闲时自动触发下一动作。
- **按键 4 随时回中**：按下后立即终止循环、清空动作队列并回归初始位置。

### Fixed

- **按键电平抖动导致机械臂误动**：修复按键状态检测函数中电平跳变引发的误触发问题；修复后摇杆控制恢复正常。
- **按键 1 按下后机械臂乱跳**：定位为动作函数阻塞问题所致，通过上述非阻塞重构一并解决。
- 夹爪初始位置由 `90` 修正为 `135`，并将 Left 舵机行程下限 `MINLpos` 由 `35` 调整为 `20`，扩展动作可达范围。

## [0.1.0] - 2026-09-29

首个里程碑版本：完成机械臂基础控制与摇杆操控，建立版本管理与开发文档体系。

### Added

- **串口通讯与指令控制程序** `3126000394mearm/3126000394mearm.ino`
  - 单字符指令：`O` 张开夹爪、`S` 闭合夹爪、`H` 加速、`L` 减速；
  - 多舵机协同控制：解析 `x(角度),y(角度),z(角度)` 指令并同步驱动三路舵机（`x→Middle`、`y→Right`、`z→Left`）；
  - `A` / `B` / `C` 自动夹取动作组；
  - 平滑插值转向函数 `turn()`，按 `speed` 逐度移动，避免舵机跳变。
- **摇杆控制程序** `yaoganmearm/yaoganmearm.ino`
  - 双摇杆（A0–A3）实时操控四个舵机（Middle / Left / Right / Claw）；
  - 摇杆死区（`JOY_DEADZONE`）与限速（`JOY_INTERVAL`、`JOY_STEP`）处理，抑制抖动；
  - 摇杆按键（A4）触发动作；
  - 按键**非阻塞去抖**状态读取（`updateButton()`）。
- **文档**
  - `机械臂设计思路.txt`：机械臂结构拆分（摇杆 / 按键 / 4 舵机）与功能实现方案；
  - `开发记录.txt`：开发过程原始记录；
  - `docs/DEVLOG.md`：结构化开发记录（追加式）；
  - `docs/开发总结.md`：开发阶段总结。

### Fixed

- 夹爪闭合角度 `CLOSECLAW` 由过小的原值修正（原值小于最小夹爪行程会持续堵转）。
