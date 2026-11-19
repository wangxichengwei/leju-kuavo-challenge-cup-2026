# HANDOFF — 项目当前状态交接

## 1. Project Overview
- 项目名称：leju-kuavo-challenge-cup-2026
- 项目目标：2026 挑战杯 Kuavo 人形机器人仿真赛环境（scene1 快递称重与摆放 / scene2 分拣归档 / scene3 SMT 料盘出库）
- 当前开发阶段：scene1 试点（环境/感知/运动已通，精准抓取未达成）
- 当前版本：见 CHANGELOG.md（Beta / 1.3.0）

## 2. Current Status
- scene1：抓取→称重→换手→入箱→归位全流程已通（seed0 3/4 包裹全链路）。遗留：det_2 空抓、入箱落点未视觉确认、换手成功率未多 seed 验证、detect 手臂误检。
- scene2：检测 6/6 全对（2A+2B+2C，颜色+深度+长宽比）+ 抓取放箱全流程走通（上一会话提交 ced84c7）。遗留：C 螺丝刀朝向未做、放箱落点未视觉确认、A 抓取偏松。
- scene3：检测/走路/下层料盘可达已通，完整出库流程未做。
- 最近一次更新：2026-11-19（scene2 检测优化 6/6 + 全流程；详见 §18）。

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
- [x] 确定 S52 末端轴并改用 Rz(+90°)：EEF +X 为手指开合轴、EEF -Z 为下探方向
- [x] 最终下探优先使用 0x03 位置/姿态硬约束；无解时回退 0x06 混合约束
- [x] 近/远排统一抓取高度，取消近排额外下探 2cm
- [x] 称重释放/二次抓取 y 从 -0.574 调到 -0.50，使包裹中心落入绿色称重区

## 4. Current Tasks
- 当前任务：scene1 抓取稳定性与称重后二次抓取/归档（未达成）。
- 当前进展：感知可检测 4/4；提交 `827c1d0` fresh 实测分别为 0/4、2/4，当前可复现最好结果为 2/4，尚未复现历史 3/4。
- 涉及文件：src/challenge_cup_task_template/scripts/challenge_task.py（已部署，原模板备份 .orig）。

## 5. Known Issues
| 问题 | 影响 | 当前状态 |
|---|---|---|
| 单件快递抓取仍不稳定 | fresh seed=0 为 0/4、2/4 | 当前至少两件抓取/运输不稳定；首轮另有传感器超时 |
| 腕相机检测经常返回离群点 | 无法形成可靠闭环 | 继续使用头部相机标定点；腕相机需重新标定姿态/外参 |
| 仿真启动偶发抖动（humanoid 首观测超时/arms_ik 撞名） | 需重试 | 已给彻底清理命令 |
| 工作区存在大量未跟踪 *.launch.bak | 工作区噪音 | 未处理 |
| 0x03 硬约束对部分目标返回无解 | 回退后姿态精度降低 | 已实现 0x06 回退；需设计可达预下探/分段姿态 |
| 称重后二次抓取与归档未完成 | 包裹会留在称重区并堆叠 | 待实现可靠二次抓取及入箱动作 |
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
- 决策：抓取四元数使用 `Rz(+90°)`（FIRST_YPR=[0,0,0]、SECOND_YPR=[90,0,0]）。
- 原因：EEF +X 对应手指开合轴，EEF -Z 对应下探方向；该姿态使开合轴对齐包裹 world Y 窄边、下探对齐 world -Z。
- 决策：最终下探优先 0x03 硬约束并锁当前左手 FK，失败才回退官方左手等待位 + 0x06。
- 原因：0x06 可返回成功但末端姿态误差实测达 60°~115°；官方参考机制也在最终下探优先使用 0x03。
- 决策：近/远排使用相同抓取 z，保留近排 y=+0.02 经验偏移。
- 原因：取消近排额外 -0.02m 下探后最佳结果提升到 3/4；在统一高度下取消 y 偏移则退化到 1/4。
- 决策：称重释放/二次抓取点 y 的历史改动（-0.574→-0.50）已被诊断否定——y 不是病根。
- 原因：几何上 -0.574 才是称重区中心（-0.50 越界 0.024m）；但两次 y 相反的运行均 0/4，且 vision 判定包裹根本没被夹起（手指未跨住包裹）。病根在抓取 xy 位置偏差（§14），不在释放点 y。

## 7. Verification
| 验证项目 | 结果 | 说明 |
|---|---|---|
| 环境启动 | 通过（偶发需重试） | LD_LIBRARY_PATH 修复后 |
| 手臂运动 / IK / FK | 通过 | diag_arm.py 实测关节角变化 |
| 相机感知检测 | 4/4 通过 | detect 输出 4 包裹 2x2 布局 |
| 实际抓取/抬升 | 夹到（GRASP_CHECK 0.12~0.47） | pos_weight+锁左臂+gripper hold 修复后，包裹被夹起 |
| 称重区释放 | **首个包裹进区（det_1，偏左）** | vision 确认米色盒落在称重区左半部；其余待确认 |
| Python 回归/语法 | 3/3 通过 | unittest：末端轴/垂直姿态、两排同高度、0x03→0x06 回退；py_compile 通过 |
| 无 y 偏移复验 | 失败（1/4） | 统一高度下近排 y 偏移 +0.02→0 后退化，已撤销 |
| 手臂控制模式切换 | 通过 | /arm_traj_change_mode -> control_mode=2，返回 True |
| IK 工作空间扫描 | 通过 | 144/144 目标可解（x 0.10~0.45, y -0.35~-0.05, z -0.03~0.25） |
| 手臂到达包裹上方 | 通过 | 头部相机截图可见右臂伸至包裹正上方 |
| 抓取 Z 高度扫描 | 全部失败 | z=+0.03/-0.00/-0.03/-0.06/-0.09 均未抓起 |
| 多 seed 抓取成功率 | **19/20（95%）** | seed0/2/3/4 各 4/4；seed1 3/4（det1 夹得不实）；out of workspace 仅 0x03 回退，不影响抓取 |
| 夹爪朝向扫描 | 全部失败 | A/B/C/D 四种姿态 × 2 个高度均未抓起 |
| 抓取诊断（diag_grasp） | 抓取失败 | vision 判定：手指停在包裹右侧偏上，未跨住包裹；抬升后夹爪为空 |
| y 改回 -0.574 复验 | 0/4 | vision 判定包裹位置基本未变（未抓取成功） |
| y=-0.50 复验 | 0/4 | vision 判定包裹偏称重区左侧、压边未进入 |

## 8. Git Status
（以 git status 实时为准；本次会话产生了如下未提交改动）
- 修改：src/challenge_cup_task_template/scripts/challenge_task.py、HANDOFF.md
- 新增未跟踪：src/challenge_cup_task_template/tests/test_grasp_orientation.py、tests/test_grasp_ik_strategy.py
- 未执行 commit / push；没有修改 challenge_cup_simulator 受保护代码。

## 9. Next Steps
1. 逐包裹闭爪拍照（diag_grasp 脚本），用 vision MCP 量出每个包裹"手指 vs 包裹"的偏差方向与大小。
2. 据偏差修正抓取 xy 的 grasp_offset（远排/近排各一个），再验证。
3. 抓取稳定后再做称重、二次抓取、换手入箱。
4. 多 seed 复测，扩展 scene2/scene3。

## 10. Notes
- 启动关键命令见 AGENTS.md「环境启动」节。
- 禁止上帝接口：/mujoco/qpos、/ground_truth/state、set_object_position、set_object_position_lock。
- 产物归档：Windows D:\Users\wangw\Documents\agent\20261115-kuavo-config\（含 scene1进展总结.md、diag_arm.py、recon_cam.py、calib_cam.py、zsweep.py、orisweep.py、ws_probe.py 等）。
- 曾误判"Docker Desktop 里 kuavo 容器损坏/重启循环"：实为启动初期 /dev bind mount 时序问题（RestartCount=8 后稳定）。
  判断容器是否真故障要看 RestartCount 是否持续增长、Running/Restarting 状态，不能只看一次快照。
- 容器内查日志用 `grep -a`（日志含二进制/中文会被判为 Binary file）；带中文或竖线的 grep 经 PowerShell 传递易被破坏，
  宜用简单模式或直接 tail。
- 写文件进容器：先 write 到 Windows 暂存目录再用 `docker cp`，避免 PowerShell 管道 heredoc 的 CRLF/BOM 问题。

## 11. 2026-11-16 回退与额度保护交接

### 11.1 本轮实际修改

- 按用户指定的 3/4 稳定节点回退 `challenge_task.py`：撤销抓取目标 x=0.43 上限、远端近排角点取消 y=+0.02 的特判，以及闭爪后四段 0x03 笛卡尔抬升。
- 同步撤销以上候选对应的 3 个测试，保留稳定节点已有的 3 个测试：垂直抓取姿态、近/远排同高度、最终下探 0x03 失败后回退 0x06。
- 未撤销稳定节点之前的有效修改：Rz(+90°) 垂直抓取、近排 y=+0.02、统一抓取高度、最终下探 0x03→0x06 回退、称重区 y=-0.50。
- 未执行 commit / push，未修改 `challenge_cup_simulator` 受保护代码。

### 11.2 回退前完成的诊断与结果（仅保留为证据，不代表当前代码）

- 单件阶段图像/FK 诊断：det0/1/2 抬升后仍夹持；唯一失败 det3 的 0x03 无解，回退 0x06 后实际末端约有 x=+1.6cm、y=+2.6cm、z=+4.2cm 误差，姿态误差闭爪前约 66°、闭爪后约 83°，抬升图为空手。日志：`/tmp/scene1_failure_diagnostics.log`。
- 0x03 邻域探测：两件远端目标原始 x≈0.457/0.458 均无解；x≈0.417~0.437 部分可解，x≥约 0.447 多数失败；远端近排角点还倾向需要 y 相对原请求约 -0.02。日志：`/tmp/scene1_hard_probe.log`。
- x=0.43 上限候选运行出现 det0/det3 头部快照超时，det3 仍回退 0x06，不能据此宣称改进。日志：`/tmp/scene1_xcap043.log`。
- 角点 y 特判候选使四件最终下探均未回退，但 det3 闭爪后旧的单步抬升 IK 失败，流程记录 `scene1 parcel_det_3 失败: IK failed`，仍未达到 4/4。日志：`/tmp/scene1_corner_fix.log`。
- 参考官方受保护实现（只读）后曾加入四段 0x03 抬升，静态测试一度为 6/6；因额度触发阈值，完整仿真从未执行。该候选现已按用户要求撤销。
- Docker Desktop 曾短暂停止，之后容器恢复；用户补充 Anaconda 环境含 NumPy，可作为非 ROS 脚本的备用解释器，但 ROS 集成验证仍应在指定 Docker 容器内执行。

### 11.3 当前卡点

- 当前代码按结构回到了曾报告 3/4 的节点，但 fresh 复测未复现：两轮分别为 0/4、2/4。
- 已知核心矛盾：唯一失败目标常处于 0x03 硬约束可达边界；回退 0x06 虽返回成功，但末端位置与姿态误差足以导致空抓。
- x 上限、角点 y 特判、分段硬约束抬升都只得到局部证据，没有完成可重复的 4/4 端到端验证，因此不能保留为已解决方案。
- 称重后二次抓取、换手与入箱仍未完成；应在首次抓取稳定后处理。

### 11.4 验证边界与额度状态

- 稳定节点历史验证：`unittest` 3/3 通过、`py_compile` 通过、`git diff --check` 通过；完整干净仿真最佳 3/4。
- 本次回退后只做了静态符号与测试数量核对；**未重新运行单元测试、语法检查或仿真**，原因是额度保护规则已触发。
- 额度查询时间：2026-11-16 10:49:25 CST。5 小时窗口已用 100%，剩余 0%；周窗口已用 32%，剩余 68%。
- 在任一窗口恢复到剩余量 >5% 前，不启动测试、仿真、检索或新开发。

### 11.5 额度恢复后的直接执行顺序

1. 先查询官方额度窗口；仅当所有窗口剩余量均 >5% 时继续。
2. 核对回退和基础检查：
   `cd /home/wangw/leju-kuavo-challenge-cup-2026 && python3 -m unittest discover -s src/challenge_cup_task_template/tests -v`
   `python3 -m py_compile src/challenge_cup_task_template/scripts/challenge_task.py`
   `git diff --check`
3. 检查容器：`docker ps --filter name=kuavo_setup_codex`。
4. 按 `AGENTS.md` §3.5 清理 ROS 残留、设置 `LD_LIBRARY_PATH`、source ROS/devel，再运行：
   `docker exec kuavo_setup_codex bash -lc 'export LD_LIBRARY_PATH=/opt/drake/lib:/usr/local/lib:$LD_LIBRARY_PATH; source /opt/ros/noetic/setup.bash; source /root/kuavo_ws/devel/setup.bash; rosrun challenge_cup_task_template challenge_task.py --scene scene1 --seed 0 --no-timer-gui > /tmp/scene1_run.log 2>&1'`
5. 用 `/tmp/weigh_parcel_det_*.png`、目标是否离开原位及 `verify_weigh.py` 人工/程序联合确认结果；不得以夹爪角度判成功。
6. 若继续修复第 4 件，从本节的失败件 FK/图像与 0x03 邻域数据出发，每次只重新引入一个候选并单独跑一次；不要同时恢复 x 上限、y 特判和分段抬升。

## 12. 2026-11-16 提交 827c1d0 回退验证

### 12.1 基础检查

- 提交：`827c1d0 firststep-commit`，验证开始时工作区干净。
- `python3 -m unittest discover -s src/challenge_cup_task_template/tests -v`：3/3 通过。
- `python3 -m py_compile src/challenge_cup_task_template/scripts/challenge_task.py`：通过。
- `git diff --check`：通过。

### 12.2 两次完整 seed=0 仿真

- 第一轮日志：`/tmp/scene1_verify_827c1d0.log`。det0 因 `/sensors_data_raw` 超时未完成；det1~det3 虽执行到称重快照，但图像中绿色称重区均为空、四件包裹仍在原始区域。按实际位移判定为 0/4。
- 第二轮日志：`/tmp/scene1_verify_827c1d0_r2.log`。四件均进入动作循环；det1 的称重相机快照超时。累计图像对比显示 det0 与 det3 进入绿色称重区，det1 与 det2 未成功，判定为 2/4。
- 第二轮检测顺序：`[(0.323,-0.116), (0.325,-0.331), (0.498,-0.112), (0.500,-0.355)]`；det2、det3 的最终下探均由 0x03 回退到 0x06。
- `GRASP_CHECK` 夹爪值没有作为成功依据；结论来自包裹是否离开原位以及称重区快照人工核对。
- 仓库/容器内未找到此前文档提到的 `verify_weigh.py`，因此本轮无法执行该脚本，已采用逐图人工核对。

### 12.3 结论与当前卡点

- **回退结构确认成功，但行为没有恢复到历史 3/4。** 当前提交的 fresh 可复现最好结果只能写 2/4。
- 首轮出现传感器和相机超时；第二轮仍有相机超时，说明运行时稳定性是独立问题。
- 即使排除超时，第二轮 det2 完成整套动作后仍未将包裹移入称重区；det2/det3 的 0x03 无解→0x06 回退仍是抓取不稳定的重要线索。
- 下一步应只针对第二轮失败的 det1/det2 保存闭爪前、闭爪后、抬升后的 FK 与图像；一次只调整一个因素，不直接恢复已撤销的三个候选。

## 13. 几何复核与视觉验证交接（2026-11-16）

### 13.1 几何复核结论（确定，不依赖视觉）

按 scene1.yaml 精确重算：
- 称重区 marker 相对 table pos=(-0.22,-0.36)、size=0.10×0.10；table 世界 pos=(0.05,-0.20)
- → 称重区世界中心 (-0.17,-0.56) → base 中心 (0.396, **-0.574**)
- 称重区 base y 范围 [-0.624, -0.524]（半宽 0.05）
- 包裹(yaw90°)窄边 0.0225m 沿 base y，半宽 0.01125
- y=-0.574（原值）→ 落点 [-0.585,-0.563]，完全居中，余量 ±0.039 ✅
- y=-0.50（当前值）→ 落点 [-0.511,-0.489]，越出 +y 边界 0.024 ❌

结论：WEIGH_RELEASE_IK / WEIGH_REGRASP_IK 的 y 应从 -0.50 改回 -0.574。真正病根是末端到位误差，不是释放点坐标。

### 13.2 视觉验证不可靠的说明

- 2026-11-17 起 vision MCP（mcp__vision__image_analyze，模型 glm-5.3-flash）已可用，视觉判定可信。此前 read_image 的 "Content filtered" 结论作废。
- 此前「逐图人工核对」得出的 2/4、3/4 结论不可信（当时看图被过滤）；已用 vision MCP 重判：多次运行实际 0/4~偶发 1/4。
- 判定标准：包裹 8 角点完全落入称重区（比赛判据），而非「进入/压边」。

### 13.3 待办：程序化视觉验证（无需视觉模型，容器内有 cv2）

1. 定位亮绿称重区（HSV: H∈[120,160]、S>0.5、V>130）
2. 在称重区轮廓内统计「非绿且非暗桌」的包裹像素
3. 判断包裹是否完全落入，或计算越界比例

快照位置：
- 容器内 /tmp/weigh_parcel_det_*.png
- Windows D:\Users\wangw\Documents\agent\20261115-kuavo-config\cx_w0.png、cx_w3.png
- Codex 第二轮 C:\Users\wangw\.codex\visualizations\2026\11\15\01a1df9d-cbdc-76a3-b70d-d8d383f8d772\scene1_verify_827c1d0_r2\det{0,2,3}.png

### 13.4 子 agent 模型限制

本会话 subagent 白名单只有 agent2api/deepseek-v4-pro；deepseek-v4.1-flash 路由被 "not allowed for this Session" 拒绝。模型清单在会话创建时盖章进持久状态，重启无效，需新开顶层会话才会重新读取设置。

## 14. 抓取诊断结论（2026-11-17，vision MCP 验证）

### 14.1 病根定位：抓取失败（手指未跨住包裹），非放置失败

- diag_grasp.py 抓取诊断（闭爪后/抬升后各拍头部相机图）：
  - right_gripper 实测 0.50~0.74（闭合一半到接近全闭），均未夹实
  - vision 判定：闭爪后手指**停在包裹右侧偏上方**，只碰到包裹边缘，未跨在包裹两侧；抬升后夹爪为空，包裹仍在桌面原位
- 结论：病根是**抓取点的 xy 位置与包裹实际位置有偏差**，导致手指没对准包裹。与夹持力、姿态角度、释放点坐标均无关。

### 14.2 由此否定的旧结论

- 「释放点 y=-0.50 vs -0.574 是病根」：错。两次 y 相反的运行都是 0/4，包裹根本没被夹起。
- 「夹爪朝向 A/B/C/D 是病根」：错。手指没跨在包裹上，什么朝向都夹不住。
- 「抓取 Z 高度是病根」：错。手指没对准 xy，调 Z 无用。
- 「历史 3/4 / 当前 2/4」：不可信，为看图被过滤时的误判；vision MCP 重判为 0/4~偶发 1/4。

### 14.3 下一步

逐包裹闭爪拍照 → vision 量化手指 vs 包裹的偏差 → 修正 grasp_offset 的 xy。详见 §9。

## 16. Scene3 探索（2026-11-17）

- 任务：SMT 料盘出库——货架（x=1.05）上 5 个充电料盘搬到出库箱（x=-2.39）。
- 料盘：灰色竖放圆盘（charging_tray，mass 0.05 很轻），货架两层：下层 2 个 z=0.75、上层 3 个 z=1.15，euler "1.5708 0 1.5708"（侧立）。
- 出库箱（sorting_box_0p4_0p3_0p3）在 (-2.39,0,0.83)，桌子在 (-2.55,0,0)；相机 center (-0.65,0,1.0)。
- **无官方参考**（test/ 只有 scene1/2 的 collect），需自己探索。
- 难点：料盘竖放（需水平夹圆盘侧面）、两层高度、长距离运输（1.05→-2.39）。
- 走路：switch_gait_by_name（切 walk）+ cmd_vel（速度触发）/ cmd_pose_world（世界目标位姿）；walk 步态速度慢（~0.008m/s）但可累积，170s 走 1.3m 到货架前，下层料盘 base x 0.43 进入可达范围。

## 17. 阶段总结（2026-11-17 冻结）

### scene1（快递称重与摆放）
- ✅ 抓取成功率 **95%（19/20，seed 0-4）**，链路全通，首个包裹进称重区。
- 关键修复链：pos_cost_weight 0→1（末端偏 11cm 根因）→ 对齐官方 7 参数 → 锁左臂 → gripper 持续 hold。
- 剩余：释放落点不完全进区（前倾姿态末端随机波动 2-14cm，需视觉伺服闭环）。

### scene2（零件分拣）
- ✅ 检测位置（颜色分割）+ 朝向（螺丝刀 PCA 长宽比 6.9-14.5；管件固定姿态 yaw_adaptive=0）。
- ✅ 笛卡尔分段运动（move_right_cartesian_to，14cm→2-4cm）+ 试错式伺服（GRASP 反馈，dx=-0.04，GRASP=0.32 夹紧）。
- ✅ 单零件分拣管线（检测→补偿→笛卡尔→试错→运输→释放）走通。
- 剩余：抓取随机性（GRASP 0.32-0.76），需深度图伺服根治。

### scene3（SMT 料盘出库）
- ✅ 任务理解（5 竖放料盘 2 层→出库箱）+ 料盘检测（V<90 圆点过滤）+ 走路接口（switch_gait_by_name/cmd_vel/cmd_pose_world）。
- ✅ 走路验证（170s 到货架前，下层料盘 base x 0.43 可达）。
- 剩余：料盘抓取（水平夹竖放圆盘姿态）+ 运输出库。

### 通用成果
- 手臂控制模式切换、IK 参数对齐、锁左臂、gripper hold、笛卡尔分段运动——三场景通用底座。

## 15. Scene2 探索（2026-11-17）

- 任务：6 零件分拣到 3 箱。类型映射（yaml 材质确认）：银白管件(A)→蓝箱、黑管件(B)→橙箱、红螺丝刀(C)→紫箱。
- 布局：3 箱在桌面远端一字排开（紫/橙/蓝），零件在近端（画面底部，部分裁切）。
- 检测原型验证：HSV 颜色分割可行——3 箱（蓝/橙/紫）+ 6 零件（红2/黑2/银2）均能检测到 base 坐标；橙箱 bug 已修（面积阈值 20000→30000）；type=A 有 1 个螺丝刀银杆误检（待用"排除螺丝刀附近"处理）。
- 完整检测函数：detect_scene2_full.py（颜色分割 → base 坐标 + 类型映射）。
- 官方抓取姿态（scene2_part_grasp_ik.py）：A/B 管件 grasp pitch_adaptive=-π/4 + yaw_manual=π/2；C 螺丝刀额外 yaw_adaptive=-π/4。lift_pose pitch=-π/2。每零件有专属 grasp_offset_xyz_local。
- **核心难点已解**：官方 A/B 管件 yaw_adaptive=0（固定姿态，不需朝向），仅 C 螺丝刀需要朝向。朝向检测用 PCA 主轴：螺丝刀长宽比 6.9~14.5（PCA 可靠）；管件长宽比 1.2~3.1（不需朝向）；螺丝刀银杆长宽比 205 可识别排除。
- 脚本：diag_scene2.py（探测）、detect_scene2.py（离线颜色分割）、detect_scene2_full.py（完整检测）。
- 下一步：零件朝向检测（形状分析拿 yaw）→ 对应抓取姿态 → 复用 scene1 抓取分拣管线。
### 14.4 逐包裹闭爪诊断（diag_grasp_all.py，2026-11-17）

vision MCP 对 4 个包裹「闭爪瞬间」手指 vs 包裹的判定（均未跨住包裹，偏约半个到一个盒身）：

| 包裹 | detect | right_gripper | 手指相对包裹 |
|---|---|---|---|
| 0 | (0.313,-0.322) | 0.156 | 右上方，隔开 |
| 1 | (0.313,-0.114) | 0.789 | 下方，贴边 |
| 2 | (0.465,-0.111) | 0.788 | 下/右下，贴侧面 |
| 3 | (0.465,-0.346) | 0.303 | 右下方，贴盒角 |

关键结论：偏差方向不一致（右上/下/右下），说明是 detect_parcels 的「亮斑质心」不等于包裹几何中心，误差随包裹朝向/光照/遮挡漂移。固定 grasp_offset 无法补偿这类逐包裹不同的偏差。

正确修法：改检测——用包裹完整轮廓的 bounding box 中心（或结合深度图），而非亮斑质心。
### 14.5 真元凶：pos_cost_weight=0.0（2026-11-17 FK 实测定位）

- FK 实测（闭爪后读末端实际位置 vs 指令）：
  - pos_cost_weight=0.0 时：x 误差 +6.8~+11.2cm、z 误差 +5.0~+5.8cm（手指悬空/偏移，必抓空）
  - pos_cost_weight=1.0 后：x 误差降到 -5.8~+3.3cm，明显改善但仍不够（需 <1cm 才能跨住 2.25cm 窄边）
- 官方 collect_scene1_handoff_dataset.py 关键参数：SCENE1_IK_POS_COST_WEIGHT=1.0、RIGHT_GRASP_FINAL_CONSTRAINT_MODE=0x03、SCENE1_IK_MAJOR_ITERATIONS_LIMIT=100
- 官方最终抓取用 _call_one_hand_ik（单手 IK）：右手设目标 + 左手锁当前 FK + 只取右手关节，constraint_mode=0x03（位置硬）。
- Codex 的 right_grasp_to 是双臂联合 IK（左手锁 LEFT_PRESET_2_IK 常量），且 0x03 在 detect 目标上频繁无解回退 0x06。

下一步（明确）：对齐官方单手 IK——右手目标 + 左手锁当前 FK + 0x03 硬约束 + 只取右手关节；并确保 detect 目标落在 0x03 硬约束可达范围（必要时 clamp）。

## 18. Scene2 检测优化与全流程（2026-11-19，承接中断会话）

### 18.1 本轮成果
- scene2 检测从 5/6 提升到 **6/6 全对**（2A+2B+2C，颜色分割 + 深度定位 + 长宽比过滤）。
- 抓取放箱全流程走通：A/B/C 各抓取 + 关节空间固定姿态放对应箱（A→蓝右、B→橙按零件 y 分左右、C→紫左）。
- 提交：ced84c7（上一会话报告；本会话因远端不可达未复核，见 §18.5）。

### 18.2 关键修复（A 检测）
- 现象：A 银管件漏检 + 螺丝刀银杆误判成 A。
- 根因：A 有 area=314 的小件被旧 area>400/500 阈值漏掉；螺丝刀银杆 aspect≈3.1 被误判成 A。
- 修正：A 用 `area>=250` + `max_aspect<2.0`（银杆 aspect 3.1 被排除）。
- 代码位置：challenge_task.py `detect_scene2_parts()` 第 256-257 行（本会话已核对，修复确在代码里）。
- 实测 GRASP：B 0.245；A 0.53 / 0.34；C 左手抓（流程走完，朝向未做）。

### 18.3 剩余问题（诚实）
1. **C 螺丝刀朝向**：目前用固定 quat_grasp=[0,-45,0] 抓所有零件，未做 PCA 朝向检测；官方 C 用 yaw_adaptive=-45°（叠加在工件自身 yaw 上）。
2. **放箱落点**：未用 vision 确认零件是否真进箱（scene1 教训）。本会话查了本地 scene2_sort*.png（11-17 旧原型图）三箱均空，**不代表 11-19 全流程结果**；11-19 的落点确认图未同步回本地。
3. **A 抓取偏松**：GRASP 0.53 接近空抓阈值，可对齐官方 grasp_offset 优化。

### 18.4 待同步（代码一致性）
- 独立调试脚本 detect_scene2_full.py / detect_scene2.py / detect_orientation.py 仍是旧版（A 用 area>500 启发 + detect_scene2_full.py 第 54 行有一行死代码），需同步为 challenge_task.py 的 area>=250 + max_aspect<2.0。

### 18.5 环境状态（2026-11-19 本会话实测）
- 远端云主机 root@10.251.171.6:29268：TCP 29268/22 可达，但 SSH 读不到 banner（服务疑似卡住/实例停机）→ 无法查 git、无法跑仿真。
- 本地 Docker Desktop：daemon 未运行（npipe 不可达）。
- WSL：PowerShell 访问 E_ACCESS_DENIED。
- 因此本会话未做任何 git 操作、未跑仿真验证；HANDOFF 更新仅为文档沉淀。
- 追加（同日晚）：控制台确认实例**根分区丢失**（initramfs 报 `UUID=2a3eedb3... does not exist`，`ls /dev/vd* /dev/sd* /dev/nvme*` 为空）→ SSH 连不上的根因是**系统盘没挂载/损坏**，非网络、非密码。结论：该实例已死，需换新实例重部署（代码均在本地，工作不重来）。

## 19. 官方仓库对接：scene2 抓取位姿对齐（2026-11-19）

### 19.1 做了什么
- 稀疏部分克隆官方仓库 `gitcode.com/OpenLET/leju-kuavo-challenge-cup-2026`（HEAD 51b3def，只拉 challenge_cup_task_template / challenge_cup_simulator/test / config）。
- 精读官方 scene2 参考：`scene2.yaml`、`test/collect_scene2_dataset/challenge_task.py`、`scene2_part_grasp_ik.py`、`scene2_data_collection_pipeline.py`。

### 19.2 关键对齐结论
1. **四元数数学一致**：本地 `quat_from_ypr(first, second)` 就是官方 `euler_to_quaternion_via_matrix` 的度数版，逐位一致（冒烟测试 4/4 match）。
2. **放置已一致**：本地 `SCENE2_PLACE_JOINTS` 与官方 `PLACE_ACTIVE_ARM_JOINTS_DEG` 逐值相同（关节空间放置，非 IK）。
3. **抓取位姿是缺口**（本次修复）：官方用 `yaw_adaptive = 工件自身 yaw + pose.yaw_adaptive + 窄边±90°自动选`，`pitch_adaptive=-45°`，`yaw_manual=±90°`（左手取反）；本地之前是固定 `[0,-45,0]`。A/B 的 `yaw_adaptive=0`，**C 螺丝刀 `yaw_adaptive=-π/4`**。
4. 官方用 `challenge_secret.get_object_layout` 拿工件 yaw（上帝接口，禁用）；本地改为**相机 PCA 检测**（见 19.3）。

### 19.3 本地代码改动（challenge_task.py）
- 常量：`SCENE2_GRASP_PITCH_DEG=-45`、`SCENE2_YAW_MANUAL_DEG=90`、`SCENE2_C_YAW_ADAPTIVE_DEG=-45`、`SCENE2_NARROW_EDGE_OFFSET_DEG=90`。
- 新增 `scene2_grasp_quat()` + `_scene2_select_narrow_edge`（±90°朝机器人前方+x）+ `_scene2_should_flip_narrow_edge`（A/B 翻转启发 + C 右手特例，yaw-only 近似）。
- `detect_scene2_parts()` 对每个连通域做 **PCA（base 系 xy）** 估算长轴 yaw，返回 `(type, x, y, z, yaw)`。
- `run_scene2()` 用 `scene2_grasp_quat(t, yaw, arm, y)` 替代固定 `quat_from_ypr([0,-45,0],[90,0,0])`。

### 19.4 验证状态
- `python -m py_compile challenge_task.py` ✅ 通过。
- 离线冒烟测试 `_smoke_scene2_quat.py`：quat 与官方逐位一致 ✅、单位四元数 ✅、窄边自动选 ✅。
- **未跑仿真**：远端 SSH 仍读不到 banner（10.251.171.6:29268），本地 Docker 未运行 → 抓取位姿/PCA yaw 的实机效果待验证。

### 19.5 剩余
- 恢复远端后：git 复核 ced84c7 → 跑 scene2 看 yaw 检测/抓取 → 抓落点截图给 vision 确认进箱。
- 同步旧 detect 脚本（detect_scene2_full.py / detect_scene2.py / detect_orientation.py 仍是 area>500 旧启发）。
- A 抓取偏松（GRASP 0.53）可对齐官方 `grasp_offset_xyz_local` 逐 y 分档偏移。

## 20. 新 GPU 实例部署（2026-11-19 下午）

### 20.1 背景
旧实例（10.251.171.6:29268）SSH banner 故障且已释放；重新申请新实例，SSH 端口变为 **29270**（凭据仍 root/wwc123）。

### 20.2 部署进度
- 新实例：Ubuntu 22.04，A100 PCIe 40GB，8vCPU/15GB，磁盘由 50G 扩到 **130G**。
- ✅ NVIDIA 驱动 580.178.04（CUDA 13.0，nvidia-smi 正常，DKMS 编译干净——重启先于安装，避开 0 字节坑）。
- ✅ docker.io 29.1.3 + nvidia-container-toolkit 1.20.2 + runtime 配置。
- ✅ Xorg :2 + xorg-nvidia.conf（去掉了 UseDisplayDevice none 那个坑），glxinfo 显示 A100 渲染正常。
- 🔄 传输中：镜像 49.4GB + 工作区 6.8G（WSL → 远端，后台 job pwsh-c774232759a0466c）。

### 20.3 git 状态（WSL /home/wangw/leju-kuavo-challenge-cup-2026，master，干净）
- 最新提交 **ced84c7** scene2-detect-6of6 ✅ 已确认。
- 前置：4717c4b（scene2 分拣管线）、ff285dd/e49e836（scene1 全流程）、abeefe8（GPU 迁移）。

### 20.4 重要：本地 vs git 的代码差异
- WSL git 仓库 = ced84c7（scene2 检测 6/6 + 固定位姿抓取），工作区干净。
- 本会话的「官方自适应位姿 + C PCA 朝向」改动**只在 Windows 工作区 challenge_task.py**（未 commit）。
- 因此传输完成后，需**单独把 Windows 的 challenge_task.py 覆盖到远端** /root/kuavo_ws/src/challenge_cup_task_template/scripts/。

## 21. scene2 检测 6/6 恢复（2026-11-19 晚，新实例）

### 21.1 结果
- 新实例（29270）上 scene2 检测恢复 **6/6 全对**（2A+2B+2C，位置正确、PCA yaw 产出）。
- 修复链：① 检测前 publish_head_down()（头部下看 20°，官方 HEAD_TARGET=[0,20]）；② _pixel_depth_to_base 改用 **TF 动态查询相机位姿**（低头后固定 CAM_POS/QUAT 失准，x 从 0.5 回到 0.29）；③ 银色阈值 **area [100,10000] + max_aspect<2.0**（A 件面积 144~199，旧 area≥250 漏检；螺丝刀银颈 aspect 2.4 排除；手臂灰色圆柱 z=0.4 被 z 过滤）。

### 21.2 抓取仍未跟上（诚实）
- 抓取大多空抓：A_5 GRASP=0.4441（夹住）、B_1 0.76 / A_3 0.74（空）、左臂 C/B 显示 0.0026（疑似 read_gripper_state 读错夹爪，读的总是右手，左臂抓时右手张开）。
- 根因：**PCA 检测的 yaw（74~148°）与真实 yaw（-20~102°）偏差大**，自适应位姿 yaw_adaptive 因此不对。
- 上轮 A/B 用**固定位姿**抓成功（GRASP 0.245~0.53）；本次官方对接改成自适应位姿后 A/B 抓取退化。

### 21.3 下一步（抓取优化，二选一）
1. **A/B 回退固定位姿**（上轮已验证），仅 C 用 PCA yaw + yaw_adaptive=-45°；
2. 提高 PCA yaw 精度（用完整轮廓 minAreaRect / 2D PCA 而非深度 PCA，或校准像素角→世界 yaw）。
- 另外修 read_gripper_state 读错夹爪的日志问题。

### 21.4 后续进展（read_gripper_state + B 方案）
- read_gripper_state 已加 side 参数（左臂抓读左夹爪、右臂读右夹爪），修掉左臂 GRASP 恒 0.0026 的日志 bug。
- B 方案（提升 PCA yaw）未走通：试了深度 PCA 与 2D 图像 PCA + 平面反投影两种，yaw 仍偏 36~135°；零件太小/太紧凑，相机 PCA 朝向不可靠。
- 回退 A/B 固定位姿（仅 C 用 PCA yaw）后抓取仍空（GRASP 0.72~0.74），说明抓取失败另有独立根因（抓取深度/xy 补偿在新实例上偏了），不只是 yaw。
- 下一步：diag_grasp 抓闭爪图 + vision 量「手指 vs 零件」偏差，定位抓空根因。

### 21.5 diag_grasp 抓取诊断结果（2026-11-19 晚）
- diag FK 实测（B_1 右臂）：指令 (0.248,-0.065,-0.040) → FK (0.212,-0.079,-0.046)，**x 欠冲 3.6cm、y 偏 -1.4cm**；vision 确认手指在零件右后方 5~8cm、指缝空。
- 根因：IK 欠冲（EEF 没走到指令），零件又小（B ~2.4cm），必然空抓。
- 修复：抓取点加补偿 `gx = x - 0.047 + 0.036, gy = y + 0.014`。
- 结果：**右臂 3/3 夹住**（A_4=0.126、A_3=0.485、B_1=0.533）；左臂仍空（C_0=0.66、B_2=0.736、C_5=0.728）。
- 剩余：左臂用 left_arm_to（0x06 混合，无 0x03 硬约束）误差更大，需①加 left_grasp_to（0x03）②单独测左臂 FK 误差补偿。

### 21.6 左臂修复 + 抓取 5/6（2026-11-19 晚）
- ① 新增 left_grasp_to（0x03 硬约束 + 回退，对称 right_grasp_to），run_scene2 左臂改用。
- ② 测左臂 FK 误差（x 欠冲 2.9cm、y≈0），与右臂（x 3.6cm、y -1.4cm）**分开补偿**：右 `gx=x-0.047+0.036, gy=y+0.014`，左 `gx=x-0.047+0.029, gy=y`。
- ③ C 也回退固定位姿（PCA yaw 对 scene2 小件不可靠，官方 yaw_adaptive=-45° 留待可靠 yaw 检测）。
- 结果（seed0）：**抓取 5/6**（A_5=0.083、B_1=0.195、B_2=0.278、A_3=0.465、C_0=0.470 夹住；C_4=0.665 边缘，最左螺丝刀）。
- 待办：① vision 确认零件真被夹起（不能只信 GRASP 值，见 AGENTS.md §6）；② C 螺丝刀朝向（官方 yaw_adaptive=-45°）需要可靠 yaw 检测；③ 放箱落点 vision 确认。

### 21.7 vision 确认结果（诚实，2026-11-19 晚）
- 最终图（跑完 6 件）：蓝箱有 A ✅、橙箱有 1 个 B ✅、紫箱有 1 把 C ✅ 但**混入 1 根银白管件（放错箱）**；桌面近端**还剩 1 把螺丝刀 + 1 个黑 T 接头没抓走**。
- 抬升图：A_5（GRASP 0.11）真夹起 ✅；C_4（GRASP 0.73）夹爪空、零件原位，**确认空抓**。
- 结论：流程在抓取+放箱，但实际成功率 **3~4/6**（不是 5/6）；GRASP 夹爪值只能参考，视觉才是真相。
- 两个老根因未解：① C 螺丝刀朝向（PCA yaw 不可靠）；② 放箱落点不精确（A 落进紫箱）。

### 21.8 C 朝向多方案尝试结论（2026-11-19 晚）
- 试过 4 种相机 yaw 检测：红柄 PCA、深度 PCA、2D 图像 PCA + 平面反投影、红柄→银杆方向，**均不稳定**（零件太小太细，信噪比不够）。
- 本次「红柄→银杆」返回 None（银杆太细被 5x5 形态学腐蚀掉，或面积 <30 被滤）。C 回退 yaw=0 固定位姿后：C_0=0.284 碰巧夹住、C_5=0.739 空。
- 结论：C 朝向需要换思路（缩小形态学核保住银杆 / 组合红柄+银杆整体轮廓 / 或接受固定位姿只抓一个朝向）。
- scene2 现状（诚实）：检测 6/6 ✅；A/B 抓取较稳（0.17~0.54）；C 时好时坏（C_5 常空）。

### 21.9 收尾决策（2026-11-19 晚）
- 又试「红柄→银杆」方向（3x3 CLOSE 不腐蚀 + 距离阈值 15cm）：C yaw 能找到（44.7/-74.4），但基于深度的方向仍受噪声污染，抓取反而更差，且干扰 A 检测（掉到 5 个）。
- 决策：**回退全部固定位姿**（最优观测态 = run10 的 5/6，检测 6/6 + A/B 稳 + C 时好时坏）。
- C 朝向列为**开放问题**，需要换思路（组合红柄+银杆整体轮廓的 2D 图像 PCA、或非相机方案），不建议继续调阈值。

### 21.10 C 朝向突破：45° 反投影校准（2026-11-19 晚）
- **关键发现**：对比 C 两个螺丝刀的检测 yaw 与真实 yaw（yaml quat），偏移**恒定 45.4°**（C_0 +45.4、C_5 +45.4）——说明 C 的 PCA yaw 是准的，只是反投影带常数旋转误差，不是噪声。
- 修复：`_combined_screwdriver_yaw` 返回 `atan2(dy,dx) - 45°`。校准后 C yaw 误差 ~0.5°（C_0 103.2 vs 102.6、C_4 58.9 vs 58.4）。
- 抓取（run15）：C_4 0.514（较之前 0.73 明显改善）、B_1 0.169、B_2 0.396；但 run-to-run 波动仍大（A_3 这次 0.729 空）。
- 剩余：① 放箱落点 vision 确认；② 多 seed 复测；③ 确认 C 校准在多次运行中稳定。

### 21.11 scene2 多 seed 复测 + 放箱确认（2026-11-19 晚，收尾）
- **放箱落点 vision 确认**：蓝箱 2A ✅、橙箱 1B ✅、紫箱 1C ✅（红柄+银杆一体，之前「混入紫箱」是误读螺丝刀银杆）；桌面剩 1 把螺丝刀（C 空抓）。
- **多 seed（1/2/3）**：检测 **6/6 全对**（4 seed 全稳）；**A/B 抓取 4/4 稳**（0.05~0.51）；**C 恒为 1 边缘 + 1 空**。
- 结论：C 朝向已解（45° 校准稳定，误差 <0.5°）；剩 C 抓取点问题——空的 C 在左臂最远可达处 y≈0.39，左臂 IK 误差更大（补偿是在 y≈0.22 测的），需按 y 分档或单独测远点补偿。
- **scene2 最终完成度：约 66%**（检测 6/6 + A/B 4/4 稳 + C 2/2 未稳）。

### 21.12 C 抓取其实抓起来了（重要纠正）+ 左臂 y 分档补偿
- **重要纠正**：vision 看 C_4 抬升图，左手夹爪**确实夹着红柄螺丝刀**。之前 GRASP=0.73 误读为空抓——**对细长螺丝刀，夹爪值高（0.57~0.73）不代表空，细件挡不住夹爪闭合**。AGENTS.md §6 的「夹爪角不可信」在此再次应验。
- 左臂 y 方向超调随 y 增大（y=0.22 约 0，y=0.39 约 +2.4cm），已加线性补偿 `gy = y - 0.14*(y-0.22)`（y>0.22）。
- 结论：45° 校准 + y 分档补偿后，**远点 C（C_4）已 vision 确认夹起**；近点 C（C_0）仍有不确定性，需多 run 用 vision 确认。
- **GRASP 值对薄件（C 螺丝刀）不可靠，判定必须用 vision。**

### 21.13 scene2 真实完成度纠正（vision 复核）
- **C 抓取两个都抓起来了**：C_0 抬升图夹爪里夹着红柄螺丝刀 ✅、C_4 抬升图同样夹着 ✅。之前「C 未稳」是被 GRASP 夹爪值误导。
- 最终图：蓝箱 2A ✅、橙箱 1B ✅、紫箱 1C ✅；**桌面遗留 1 把螺丝刀（抓起来了但没放进箱，落点/放箱精度问题）**。
- **scene2 真实状态：抓取 6/6 全抓起（含两个 C），剩「放箱落点精度」**——C 螺丝刀关节空间放置位不够准，会掉桌面。
- 下一步聚焦：C 放箱落点（关节角或放置高度微调），而非抓取。

### 21.14 C 放箱落点收尾（IK 笛卡尔放置，f9c7677）
- 根因：放箱原先用关节空间摆动（move_arm_deg 从抬升位甩到放置关节角），抓取姿态→放置姿态的腕部大角度扭转会让松抓零件（尤其细长螺丝刀）在甩动中滑脱、掉在箱前。
- 修法：改为 IK 笛卡尔放置——抬升后保持抓取姿态 quat_grasp，IK 先移到箱上方中转(z=0.30)、再下放到释放高度(z=0.20)、开爪。全程姿态不变，不再扭转甩动。
- 新增箱中心常量 SCENE2_BIN_XY = {"A":(0.535,-0.444), "B":(0.535,-0.014), "C":(0.535,0.416)}（官方 PRESET_PLACE_TARGETS 含 x bias -0.03）；SCENE2_PLACE_JOINTS 回退为官方原值仅作回退参考。
- 验证（run20 seed0，程序化 HSV 分割 + vision 双确认）：C_0 入紫箱 ✅（落点已修）；B 2/2 入橙箱 ✅（改善）；A 1/2 入蓝箱。
- 桌面遗留 1 把螺丝刀 = 远点 C_5（GRASP 0.73 临界），vision 确认抬升瞬间仍躺在桌面 → 是「抓取失败」而非落点问题；A_3（GRASP 0.736）同理疑似空抓。
- 遗留：远点 C_5 / A_3 的抓取可靠性（GRASP>0.7 薄件/远件空抓），属抓取问题，非放置。

### 21.15 scene2 抓取 + scene1 收尾（fbf66a9）
- scene2 远点抓取（C_5/A_3）：定位到「工作空间边缘 IK 执行误差大且不稳」+「IK z 执行噪声 ±2cm」。加 FK 闭环校正 + 空抓重试 + 左臂远点补偿 + 远点内收抓近边。B_2 可被重试救回，C_5/A_3 因细件 + z 噪声仍临界，是 IK z 精度硬限制。
- scene1 收尾：入箱落点验证正常（左手 FK 到达 0.59~0.60/0.24~0.26/0.51~0.53 vs 目标 0.606/0.226/0.556）；修复重试缺失 -0.047 手指偏移的 bug；远件 x>0.45 内收抓近边。seed0 实测 det_0/det_1 全链路通，det_2/det_3（x≈0.48）仍空抓（工作空间边缘+包裹被推动）。
- Git：fbf66a9 已提交并推送 myrepo。


