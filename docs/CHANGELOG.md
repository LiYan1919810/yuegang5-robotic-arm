# Changelog

本项目所有值得记录的变更都记录在此文件中。

格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，
版本号遵循 [语义化版本 Semantic Versioning](https://semver.org/lang/zh-CN/)。

<!-- 版本分类：Added 新增 / Changed 变更 / Fixed 修复 / Removed 移除 -->

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
