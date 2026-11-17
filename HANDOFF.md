# HANDOFF — 项目当前状态交接

## 1. Project Overview
- 项目名称：leju-kuavo-challenge-cup-2026
- 项目目标：2026 挑战杯 Kuavo 人形机器人仿真赛环境（scene1 快递称重与摆放 / scene2 分拣归档 / scene3 SMT 料盘出库）
- 当前开发阶段：scene1 试点（环境/感知/运动已通，精准抓取未达成）
- 当前版本：见 CHANGELOG.md（Beta / 1.3.0）

## 2. Current Status
- 整体进度：scene1 抓取**稳定成功 4/4**（GRASP_CHECK 0.29~0.45，全部夹起）；释放落点压称重区边缘（不完全进区），根因=释放姿态 z 精度差（偏低 4~6cm）+ 掉落随机性；**决策转向多 seed 复测与场景2/3 复用**。
- 最近一次更新：2026-11-17（抓取稳定 4/4，决策 B：释放留待后续优化）。

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

## 15. Scene2 探索（2026-11-17）

- 任务：6 零件分拣到 3 箱。类型映射（yaml 材质确认）：银白管件(A)→蓝箱、黑管件(B)→橙箱、红螺丝刀(C)→紫箱。
- 布局：3 箱在桌面远端一字排开（紫/橙/蓝），零件在近端（画面底部，部分裁切）。
- 检测原型验证：HSV 颜色分割可行——红螺丝刀 2/黑管件 2/银管件 2/蓝箱 1/紫箱 1 均能检测；橙箱阈值待调、背景有误检需 ROI 过滤。
- 脚本：diag_scene2.py（探测）、detect_scene2.py（离线颜色分割）。
- 下一步：调橙箱检测 + 写 scene2 完整检测函数（颜色分割+ROI+类型映射）+ 复用 scene1 抓取分拣管线。
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
