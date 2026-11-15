# HANDOFF — 项目当前状态交接

## 1. Project Overview
- 项目名称：leju-kuavo-challenge-cup-2026
- 项目目标：2026 挑战杯 Kuavo 人形机器人仿真赛环境（scene1 快递称重与摆放 / scene2 分拣归档 / scene3 SMT 料盘出库）
- 当前开发阶段：scene1 试点（环境/感知/运动已通，精准抓取未达成）
- 当前版本：见 CHANGELOG.md（Beta / 1.3.0）

## 2. Current Status
- 整体进度：仓库可编译可运行；scene1 试点已完成「环境启动 → 相机感知 → 手臂运动 → IK/FK → 夹爪」，手臂已能真正到达包裹上方，但「实际抓取并移动快递」尚未成功。
- 最近一次更新：2026-11-15（修复手臂控制模式与 IK 左手锁定位姿后，抓取仍 0/4，卡在抓取姿态/手指对位）。

## 3. Completed Work
- [x] 仓库已构建（build/、devel/ 存在）
- [x] Docker 镜像 kuavo_challenge_cup_2026:latest 可用（Docker Desktop 引擎）
- [x] 三场景定位（scene1/2/3 入口、YAML、评测方式）
- [x] 环境启动排障：LD_LIBRARY_PATH=/opt/drake/lib:/usr/local/lib（解决 libdrake.so / casadi / lcm 加载）
- [x] 手臂/IK/FK/夹爪全链路验证（diag_arm.py 全绿）
- [x] 相机感知：内参 + 相机位姿 + 像素→base_link 映射 + 标定（x 修正 -0.057）
- [x] 抓取调优移植：近/远身行偏移 + 收敛等待
- [x] 建立 AGENTS.md / HANDOFF.md / docs/architecture.md / plans/
- [x] 修复「手臂完全不跟指令」：调用 /arm_traj_change_mode 切外部控制模式(control_mode=2)
- [x] 修复「IK 报 Right hand out of workspace」：左臂锁定改用官方 LEFT_PRESET_2_IK=[0.313,0.239,0.282]
- [x] 修复「arm mode 服务被打崩」：改为等 /bot_stand_up_complete + 传感器就绪后再调用
- [x] 实测 IK 工作空间：x∈[0.10,0.45]、y∈[-0.35,-0.05]、z∈[-0.03,0.25] 共 144 点全部可解
- [x] 抓取 Z 高度扫描（+0.03~-0.09）与夹爪朝向扫描（4 种）已完成，均未抓起包裹

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
| 抓取姿态不对：夹爪从右后方斜向接近，手指未横跨包裹窄面 | 闭爪夹不实，0/4 | 待解决：需确定末端坐标系与手指开合轴，改用垂直向下姿态 |
| 腕部相机在预抓取位看不到桌面（朝外看墙） | 腕部视觉伺服无法工作 | 待解决：需先调姿态使相机朝向桌面 |
| 亮度阈值检测会把白色机械臂误检成包裹 | verify 结果误判 | 已加幅值门限过滤，仍需几何过滤 |
| 后台 docker exec -d 运行看不到 MuJoCo GUI 窗口 | 无法人工观察动作 | 已知：需按官方 run_with_gpu_for_challenge.sh 进容器才可见 |

## 6. Technical Decisions
- 决策：仿真用 CPU 渲染（容器内无 GPU），不影响逻辑正确性、只影响速度。
- 原因：nvidia-smi 为空，官方 --gpus 未生效。
- 决策：环境启动必须显式 export LD_LIBRARY_PATH=/opt/drake/lib:/usr/local/lib。
- 原因：镜像 .bashrc/.zshrc 未生效；bash -lc 登录 shell 不 source .bashrc。
- 决策：相机感知用头部相机 + 亮度阈值（V>165）检测快递；像素→base_link 射线求交，x 需 -0.057 标定修正。
- 决策：任务层当前为开环（感知→抓取），下一步改为腕部相机视觉伺服闭环。
- 决策：手臂必须先切外部控制模式（/arm_traj_change_mode，control_mode=2）才能被 /kuavo_arm_traj 驱动。
- 原因：默认在自动摆动模式(1)，此前所有抓取失败的第一根因即为手臂根本没到指定位置（有头部相机截图为证）。
- 决策：两臂联合 IK 时左臂固定锁在 LEFT_PRESET_2_IK（官方等待位），不锁"当前 FK 位姿"。
- 原因：锁当前 FK 会导致右臂 IK 全部返回 "Right hand out of workspace"。
- 决策：服务类调用（arm mode 等）必须排在 /bot_stand_up_complete + /sensors_data_raw 之后。
- 原因：过早调用会使服务报 transport error 并消失，打断后续所有 IK。
- 决策：抓取成功与否以"包裹是否离开原位/进入目标区"为准，不采信夹爪关节角。
- 原因：闭爪值 0.57~0.72 曾被误读为"夹住"，实际是碰到桌面或空气；称重区始终为空。

## 7. Verification
| 验证项目 | 结果 | 说明 |
|---|---|---|
| 环境启动 | 通过（偶发需重试） | LD_LIBRARY_PATH 修复后 |
| 手臂运动 / IK / FK | 通过 | diag_arm.py 实测关节角变化 |
| 相机感知检测 | 4/4 通过 | detect 输出 4 包裹 2x2 布局 |
| 实际抓取 | 0/4 | 快递未被移动 |
| 称重 | 0/4 | 称重区为空 |
| 手臂控制模式切换 | 通过 | /arm_traj_change_mode -> control_mode=2，返回 True |
| IK 工作空间扫描 | 通过 | 144/144 目标可解（x 0.10~0.45, y -0.35~-0.05, z -0.03~0.25） |
| 手臂到达包裹上方 | 通过 | 头部相机截图可见右臂伸至包裹正上方 |
| 抓取 Z 高度扫描 | 全部失败 | z=+0.03/-0.00/-0.03/-0.06/-0.09 均未抓起 |
| 夹爪朝向扫描 | 全部失败 | A/B/C/D 四种姿态 × 2 个高度均未抓起 |

## 8. Git Status
（以 git status 实时为准；本次会话产生了如下未提交改动）
- 新增未跟踪：AGENTS.md、HANDOFF.md、docs/architecture.md、plans/、docker 脚本改动
- 修改：src/challenge_cup_task_template/scripts/challenge_task.py（原模板备份 .orig）
- 未提交的既有改动：docker/run_with_gpu_for_challenge.sh + 11 个 load_kuavo_*.launch

## 9. Next Steps
1. 先解决抓取姿态：确定末端坐标系与手指开合轴，让夹爪**垂直向下**且手指横跨包裹 4.5cm 窄面；再用头部相机截图逐步验证。
2. 称重成功验证（快照/几何判定）。
3. 补入箱动作（右手→左手交接 + 左手放箱）。
4. 多 seed 复测成功率。
5. 复用管线扩展 scene2/scene3。

## 10. Notes
- 启动关键命令见 AGENTS.md「环境启动」节。
- 禁止上帝接口：/mujoco/qpos、/ground_truth/state、set_object_position、set_object_position_lock。
- 产物归档：Windows D:\Users\wangw\Documents\agent\20261115-kuavo-config\（含 scene1进展总结.md、diag_arm.py、recon_cam.py、calib_cam.py、zsweep.py、orisweep.py、ws_probe.py 等）。
- 曾误判"Docker Desktop 里 kuavo 容器损坏/重启循环"：实为启动初期 /dev bind mount 时序问题（RestartCount=8 后稳定）。
  判断容器是否真故障要看 RestartCount 是否持续增长、Running/Restarting 状态，不能只看一次快照。
- 容器内查日志用 `grep -a`（日志含二进制/中文会被判为 Binary file）；带中文或竖线的 grep 经 PowerShell 传递易被破坏，
  宜用简单模式或直接 tail。
- 写文件进容器：先 write 到 Windows 暂存目录再用 `docker cp`，避免 PowerShell 管道 heredoc 的 CRLF/BOM 问题。
