# AGENTS.md — Kuavo 挑战杯 2026 仿真项目开发规范

> 本文件是长期有效的 AI 开发指令：AGENTS.md 管规则，HANDOFF.md 管状态，Git 管代码变更历史。
> 规则来源：《长周期项目 AI Agent 开发配置规范》v1.0。

## 1. 项目概览

- 项目名称：leju-kuavo-challenge-cup-2026
- 项目目标：2026 挑战杯 Kuavo（夸父）人形机器人仿真赛环境（MuJoCo 仿真、任务场景、选手任务模板与常用接口）
- 项目类型：ROS 机器人仿真与控制（catkin 工作区）
- 主要技术栈：ROS Noetic（Ubuntu 20.04 容器）、C++ / Python、catkin_tools、MuJoCo、OCS2(MPC+WBC)、Drake、CasADi、LCM、BehaviorTree/PyTree；可选 Gazebo / Isaac Sim
- 当前开发阶段：已构建、可运行（build/、devel/ 存在；Docker 镜像 kuavo_challenge_cup_2026:latest 可用）

## 2. 关键目录结构

- src/：源码，31 个功能包（kuavo_* 系列 + ocs2 / mujoco / gazebo / humanoid-control / humanoid-wheel-control / challenge_cup_simulator / manipulation_nodes 等）
- build/、devel/：catkin 构建产物与 devel 空间
- logs/：运行日志
- docker/：镜像构建与容器启动脚本（Dockerfile、run_with_gpu_for_challenge.sh 等）
- scripts/：启动 / 环境 / 工具脚本
- docs/：docusaurus 文档站（中文产品 / 调试 / 接口文档）
- tools/、third_party_libs/、resources/、installed/

## 3. 构建 / 运行方式

- 运行环境：Docker 镜像 kuavo_challenge_cup_2026:latest
- 启动容器：在仓库根目录执行 ./docker/run_with_gpu_for_challenge.sh（设置 ROBOT_VERSION=52，将仓库挂载到容器 /root/kuavo_ws，host 网络 + X11）
- 编译：容器内工作目录 /root/kuavo_ws 执行 catkin build
- 挑战赛仿真入口：src/challenge_cup_simulator/launch/load_kuavo_mujoco_challenge.launch
- 选手任务模板：src/challenge_cup_task_template/scripts/challenge_task.py

## 3.5 环境启动（重要）

容器内运行仿真/任务前，必须先设置库路径并 source：

```bash
export LD_LIBRARY_PATH=/opt/drake/lib:/usr/local/lib:$LD_LIBRARY_PATH
source /opt/ros/noetic/setup.bash
source /root/kuavo_ws/devel/setup.bash
```

否则 humanoid 节点会因 libdrake.so / casadi / lcm 加载失败而崩溃。

启动前清理残留 ROS 进程（否则同名节点冲突、arms_ik 被撞掉）：

```bash
pkill -9 -f ros; pkill -9 -f nodelet; pkill -9 -f mujoco; pkill -9 -f humanoid; pkill -9 -f arms_ik; pkill -9 -f motion_capture
```

启动任务必须遵守的时序（踩过坑，不可颠倒）：

1. 先等机器人站起：`/bot_stand_up_complete`；
2. 再读一次 `/sensors_data_raw` 确认传感器已上线；
3. **然后才调用 `/arm_traj_change_mode` 切到外部控制模式（control_mode=2）**；
4. 最后才开始下发 `/kuavo_arm_traj`。

> 若在 `launcher.start()` 返回后立刻调用 `/arm_traj_change_mode`，该服务会报
> "transport error" 并随后消失（节点被打断），后续 IK 全部失败。必须等站起+传感器就绪。

运行任务：`rosrun challenge_cup_task_template challenge_task.py --scene scene1 --seed 0 --no-timer-gui`

## 3.6 机械臂 / IK 使用规约（重要）

- **手臂必须处于外部控制模式(2)**：默认是自动摆动模式(1)，此时 `/kuavo_arm_traj` 完全不生效。
  切换方式与时机见 §3.5（顺序错了会把服务打崩）。
- **锁左臂要用官方等待位**：两臂联合 IK 中，左手必须锁在
  `LEFT_PRESET_2_IK=[0.313, 0.239, 0.282]` + 对应四元数；锁"当前 FK 位姿"会导致
  `Right hand out of workspace!`（右臂 IK 无解）。
- **坐标系别搞混**：`base_link` 原点在世界 `[-0.566, 0.014, 0.924]` 附近，不是世界原点；
  world→IK(base) 固定偏移 `WORLD_TO_IK_OFFSET=[0.565966, -0.013886, -0.923783]`。
- **IK 参数**：`constraint_mode=0x06`（三点混合）、`major_iterations_limit=500`、
  `use_custom_ik_param=True`、`joint_angles_as_q0=True`。
- **/kuavo_arm_traj 单位是度**，14 个关节（arm_joint_1..14），需按 ~50Hz 持续发布保持，
  单次发布不会保持到位。
- **夹爪**：`/gripper/command`（JointState，left/right_gripper_joint）：**0=张开，255=闭合**；
  官方服务 `/control_robot_leju_claw` 用 0~100%。

## 3.7 感知与标定规约（重要）

- **禁止上帝接口**：`/mujoco/qpos`、`/ground_truth/state`、`set_object_position`、
  `set_object_position_lock` 均为禁用接口，反作弊监控会杀节点。官方 `test/collect_*`
  参考代码里用 `challenge_secret.get_object_layout` 的写法**只能借鉴其机械臂/夹爪/IK 机制，
  不能抄它的感知**。
- **头部相机**（`/cam_h`）：fx=fy=392.9, cx=640, cy=360, 1280x720，帧 `Head Camera View`；
  base_link→相机 pos=[0.1187,0,0.6388], quat=[0.6159,-0.6159,0.3474,-0.3474]（xyzw）。
- **像素→base**：射线与平面 z=-0.044 求交后，x 还需 **-0.057 标定修正**（y 误差 <0.016m 可接受）。
- **检测阈值会把机器人自己当目标**：亮度阈值 V>165 会把白色机械臂/夹爪误检成快递，
  统计 blob 数会虚高（实测多出 2 个）。需加几何/颜色过滤，或直接看图核对（判定规则见 §6）。
- **腕部相机**（`/cam_r`）：fx=fy=623.5, cx=640, cy=360，帧 `Right wrist Camera View`；
  在预抓取位（z=0.12）实测**看不到桌面**（朝外看墙），不能直接用于视觉伺服，
  需先调整姿态让相机朝向桌面。

## 4. 开发规范

1. 修改代码前必须先理解现有实现，并先读 HANDOFF.md 了解当前状态。
2. 优先复用已有模块和接口，禁止无理由重构已有架构。
3. 不得随意删除现有功能，不修改与当前任务无关的文件。
4. 避免引入不必要的第三方依赖，不随意升级核心依赖。
5. 保持代码风格统一（遵循 ROS/catkin 约定，C++ 与 Python 各自统一）。
6. 重要逻辑必须提供必要注释。
7. 修改尽量小范围、可验证。
8. 不确定的技术决策先向用户确认。

## 5. 代码修改规范

- 修改前：阅读相关源码、确认模块调用关系、分析潜在影响、明确本次修改范围。
- 修改时：优先最小必要修改；不擅自更改公共接口；不覆盖用户未提交的修改。
- 修改后：检查代码差异、执行相关测试 / 编译、记录修改文件及原因。

## 6. 测试 / 验证规范

- 依据修改范围执行：编译检查（catkin build）、单元测试、集成 / 功能测试、必要的人工验证。
- 测试结果必须真实记录；环境不允许时说明哪些未执行、原因与风险；禁止把未验证功能描述为通过。
- **抓取/搬运类任务不得用夹爪关节角判定成功**：闭爪值 <0.8 只说明手指碰到东西（可能是桌面、
  空气或自身），不代表夹住了目标。必须以"目标物体是否离开原位 / 是否落入目标区域"为准，
  且用相机图像人工/程序化核对（参考 `verify_weigh.py` 的做法）。

## 7. 会话交接规范

恢复本项目开发时，先读 `HANDOFF.md` 最新交接节；当前应从 §11 的 3/4 回退节点与额度恢复步骤继续，不得把已回退候选误当作当前实现。

每次重要开发会话结束时：
1. 总结本次完成的工作。
2. 记录修改的文件与原因。
3. 记录测试结果。
4. 记录未解决的问题与重要技术决策。
5. 记录 Git 状态。
6. 更新 HANDOFF.md 并给出下一步明确任务。

## 8. Git 与操作边界

- 不擅自 commit / push / 清理工作区；Git 操作需用户明确授权。
- 不删除重要源码或数据；不覆盖用户未提交的修改。
- 不将密码、Token、密钥写入 Markdown 文档。
- 不伪造测试结果或开发进度。
- 不在无依据时修改项目架构。
