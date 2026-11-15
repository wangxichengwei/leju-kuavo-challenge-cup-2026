# HANDOFF — 项目当前状态交接

## 1. Project Overview
- 项目名称：leju-kuavo-challenge-cup-2026
- 项目目标：2026 挑战杯 Kuavo 人形机器人仿真赛环境（scene1 快递称重与摆放 / scene2 分拣归档 / scene3 SMT 料盘出库）
- 当前开发阶段：scene1 试点（环境/感知/运动已通，精准抓取未达成）
- 当前版本：见 CHANGELOG.md（Beta / 1.3.0）

## 2. Current Status
- 整体进度：仓库可编译可运行；scene1 试点已完成「环境启动 → 相机感知 → 手臂运动 → IK/FK → 夹爪」，但「实际抓取并移动快递」尚未成功。
- 最近一次更新：2026-11-15（scene1 抓取调优后实测成功率仍 0/4）。

## 3. Completed Work
- [x] 仓库已构建（build/、devel/ 存在）
- [x] Docker 镜像 kuavo_challenge_cup_2026:latest 可用（Docker Desktop 引擎）
- [x] 三场景定位（scene1/2/3 入口、YAML、评测方式）
- [x] 环境启动排障：LD_LIBRARY_PATH=/opt/drake/lib:/usr/local/lib（解决 libdrake.so / casadi / lcm 加载）
- [x] 手臂/IK/FK/夹爪全链路验证（diag_arm.py 全绿）
- [x] 相机感知：内参 + 相机位姿 + 像素→base_link 映射 + 标定（x 修正 -0.057）
- [x] 抓取调优移植：近/远身行偏移 + 收敛等待
- [x] 建立 AGENTS.md / HANDOFF.md / docs/architecture.md / plans/

## 4. Current Tasks
- 当前任务：scene1 精准抓取（未达成）。
- 当前进展：感知 4/4、动作执行 4/4、实际抓取 0/4（快递未被移动，称重区为空）。
- 涉及文件：src/challenge_cup_task_template/scripts/challenge_task.py（已部署，原模板备份 .orig）。

## 5. Known Issues
| 问题 | 影响 | 当前状态 |
|---|---|---|
| 微小快递（2cm 宽）精准抓取失败 | 成功率 0/4 | 待解决：需腕部相机视觉伺服闭环 |
| 任务层「感知→抓取」是开环，位姿误差无校正 | 抓取对不准 | 待解决 |
| 仿真启动偶发抖动（humanoid 首观测超时/arms_ik 撞名） | 需重试 | 已给彻底清理命令 |
| 工作区存在大量未跟踪 *.launch.bak | 工作区噪音 | 未处理 |

## 6. Technical Decisions
- 决策：仿真用 CPU 渲染（容器内无 GPU），不影响逻辑正确性、只影响速度。
- 原因：nvidia-smi 为空，官方 --gpus 未生效。
- 决策：环境启动必须显式 export LD_LIBRARY_PATH=/opt/drake/lib:/usr/local/lib。
- 原因：镜像 .bashrc/.zshrc 未生效；bash -lc 登录 shell 不 source .bashrc。
- 决策：相机感知用头部相机 + 亮度阈值（V>165）检测快递；像素→base_link 射线求交，x 需 -0.057 标定修正。
- 决策：任务层当前为开环（感知→抓取），下一步改为腕部相机视觉伺服闭环。

## 7. Verification
| 验证项目 | 结果 | 说明 |
|---|---|---|
| 环境启动 | 通过（偶发需重试） | LD_LIBRARY_PATH 修复后 |
| 手臂运动 / IK / FK | 通过 | diag_arm.py 实测关节角变化 |
| 相机感知检测 | 4/4 通过 | detect 输出 4 包裹 2x2 布局 |
| 实际抓取 | 0/4 | 快递未被移动 |
| 称重 | 0/4 | 称重区为空 |

## 8. Git Status
（以 git status 实时为准；本次会话产生了如下未提交改动）
- 新增未跟踪：AGENTS.md、HANDOFF.md、docs/architecture.md、plans/、docker 脚本改动
- 修改：src/challenge_cup_task_template/scripts/challenge_task.py（原模板备份 .orig）
- 未提交的既有改动：docker/run_with_gpu_for_challenge.sh + 11 个 load_kuavo_*.launch

## 9. Next Steps
1. 任务层改闭环：腕部相机视觉伺服（检测快递+夹爪相对位，迭代对齐再闭爪）。
2. 称重成功验证（快照/几何判定）。
3. 补入箱动作（右手→左手交接 + 左手放箱）。
4. 多 seed 复测成功率。
5. 复用管线扩展 scene2/scene3。

## 10. Notes
- 启动关键命令见 AGENTS.md「环境启动」节。
- 禁止上帝接口：/mujoco/qpos、/ground_truth/state、set_object_position、set_object_position_lock。
- 产物归档：Windows D:\Users\wangw\Documents\agent\20261115-kuavo-config\（含 scene1进展总结.md、diag_arm.py、recon_cam.py、calib_cam.py 等）。
