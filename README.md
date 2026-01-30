项目以 Unitree Go1 四足机器人 为对象，围绕仿真、感知与控制，系统性地完成了从传统规则控制 → 视觉闭环 → 强化学习具身智能探索 的完整技术路线。

本仓库是一个 ROS 工作空间（workspace），而不是单一 ROS 包。各功能模块以 ROS package 的形式组织，位于 ros_ws/src/my_packages/ 目录下。





FOCUS-WinterCamp-2026/
└── ros_ws/
    ├── src/
    │   ├── my_packages/
    │   │   ├── go1_square_demo     # 阶段 1.2：走正方形
    │   │   ├── go1_vision          # 阶段 1.3：找红方块
    │   │   └── go1_rl_optional     # 阶段 1.4（选做）：基于雷达的强化学习
    │   └── third_party/            # Unitree 官方仿真与控制相关包
    ├── build/      # catkin 生成（已忽略，不纳入版本控制）
    ├── devel/      # catkin 生成（已忽略，不纳入版本控制）
    └── README.md   # Workspace 级说明文档
    

运行环境说明
• 操作系统：Ubuntu 20.04
• ROS 版本：ROS Noetic
• Python：3.8
• 仿真器：Gazebo
• 强化学习框架：
• PyTorch（CPU 版本）
• gymnasium
• stable-baselines3
• tensorboard
建议强化学习相关代码在 Python 虚拟环境（venv） 中运行，以避免污染系统环境。





！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！具体操作！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！


安装依赖
sudo rosdep init
rosdep update


编译
cd ros_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
建议写入环境变量



启动 Go1 仿真
roslaunch unitree_guide gazeboSim.launch


启动控制器
rosrun unitree_guide junior_ctrl
然后按2然后按5，！！！！！！！！！！！！非常重要，不然后面的演示无法正常进行！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！


走正方形
rosrun go1_square_demo move_in_square.py


追红色box(开始之前可以先手动调一下红方块的位置获得更好的演示效果)
rosrun go1_vision red_visual_servo.py


# 安装 venv（系统若已安装可跳过）
sudo apt update
sudo apt install -y python3-venv

# 创建虚拟环境
python3 -m venv ~/venvs/go1rl

# 激活虚拟环境（每次训练/运行 RL 前都要执行）或者写入环境变量
source ~/venvs/go1rl/bin/activate

# 升级 pip 工具链
python -m pip install -U pip setuptools wheel

安装 PyTorch（CPU 版本，国内源加速）装有了可以不用
pip install \
  -i https://pypi.tuna.tsinghua.edu.cn/simple \
  -f https://mirrors.aliyun.com/pytorch-wheels/cpu \
  torch torchvision torchaudio

安装 Stable-Baselines3 / Gymnasium / TensorBoard
pip install -U -i https://pypi.tuna.tsinghua.edu.cn/simple \
  "stable-baselines3>=2.0.0" \
  gymnasium \
  shimmy \
  tensorboard \
  numpy

建议把这些写入环境变量,这里的绝对路径要与自己相对应
source /opt/ros/noetic/setup.bash
source ~/FOCUS-WinterCamp-2026/ros_ws/devel/setup.bash
source ~/venvs/go1rl/bin/activate


强化学习

# 激光雷达观测预处理
rosrun go1_rl_optional scan_to_10.py

# Gym 环境随机动作验证
rosrun go1_rl_optional test_env_random.py

# PPO 训练
rosrun go1_rl_optional train_ppo_sb3.py




包功能总览
1️⃣ go1_square_demo（阶段 1.2）
基础运动控制模块
• 验证 Go1 的 /cmd_vel 运动控制接口
• 演示正方形行走等基础运动逻辑
• 为后续自动控制提供运动入口验证
￼
2️⃣ go1_vision（阶段 1.3）
基于视觉的闭环控制模块
• 在 Go1 头部挂载 RGB 相机
• 使用 OpenCV（HSV）进行红色目标检测
• 构建视觉伺服闭环：
• 目标偏移 → 角速度控制
• 目标大小 → 前进 / 停止策略
• 实现目标丢失时的搜索行为
￼
3️⃣ go1_rl_optional（阶段 1.4 · 选做）
强化学习与具身智能探索模块
• 在 Go1 上挂载 2D 激光雷达（LiDAR）
• /scan → 前方 180° → 降采样为 10 维观测
• 封装 ROS + Gazebo → Gymnasium 环境
• 使用 PPO（stable-baselines3） 进行训练
• 支持：
• 随机动作环境验证
• TensorBoard 训练曲线
• 自主避障行为演示
该模块重点探索从 规则控制向学习控制的过渡，体现具身智能思想。


设计说明
• 本仓库采用 单 workspace、多 package 的组织方式
以清晰展示课程各阶段的技术演进过程
• 强调 工程可运行性与系统稳定性
• 官方 Unitree 包被完整保留，以保证仿真环境的可复现性
• 强化学习部分以“验证方法与流程”为目标，而非追求极限性能



声明
本项目为 教学与科研用途，
是冬令营课程任务的一部分，非 Unitree Robotics 官方项目。





