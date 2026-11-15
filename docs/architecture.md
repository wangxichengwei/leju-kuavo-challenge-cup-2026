# 系统架构

## 总体
本仓库为 ROS Noetic catkin 工作区，提供 2026 挑战杯 Kuavo 人形机器人的 MuJoCo 仿真环境、任务场景与选手接口。

## 运行时架构
- 开发机：WSL Ubuntu 26.04
- 运行环境：Docker 镜像 kuavo_challenge_cup_2026:latest（Ubuntu 20.04 + ROS Noetic + MuJoCo / Drake / OCS2 / CasADi / LCM）
- 启动：docker/run_with_gpu_for_challenge.sh 将仓库挂载到容器 /root/kuavo_ws，ROBOT_VERSION=52，host 网络 + X11 显示

## 模块划分（src/）
- 仿真：challenge_cup_simulator、mujoco、gazebo、kuavo-isaac-sim
- 控制：humanoid-control（humanoid_controllers、h12pro_controller_node 等）、humanoid-wheel-control、ocs2（MPC+WBC）
- 机器人资产与基础：kuavo_assets、kuavo_common、kuavo_msgs、kuavo_sdk、kuavo_solver、kuavo_humanoid_sdk(_js / websocket)
- 外设与工具：kuavo_led、kuavo_monitor、kuavo_wheel、kuavo_mapping、manipulation_nodes、ros_audio、hand_eye_calibration、elevation_mapping、robot_localization、lcm_msg、leju_utils、demo、automatic_test

## 构建
catkin_tools 构建（build/、devel/）；C++（CMakeLists.txt 约 297 个、package.xml 约 241 个）与 Python（约 2164 个 .py）混合。
