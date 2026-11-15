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

运行任务：`rosrun challenge_cup_task_template challenge_task.py --scene scene1 --seed 0 --no-timer-gui`

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

## 7. 会话交接规范

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
