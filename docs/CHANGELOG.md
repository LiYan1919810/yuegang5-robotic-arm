# Changelog

本项目所有值得记录的变更都记录在此文件中。

格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，
版本号遵循 [语义化版本 Semantic Versioning](https://semver.org/lang/zh-CN/)。

<!-- 版本分类：Added 新增 / Changed 变更 / Fixed 修复 / Removed 移除 -->

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
