### 环境
- ubuntu20.04
- ros2 foxy

```tree
panda_ws/
├── build/                         # 编译构建目录
├── install/                       # 安装目录
├── log/                           # 日志目录
└── src/                           # 源代码目录
    │
    ├── PandaRobot/                # Panda 机器人主包
    │   ├── README.md              # 项目说明文档
    │   │
    │   ├── panda_ros2_gazebo/     # Gazebo 仿真包
    │   │   ├── CMakeLists.txt     # CMake 构建配置
    │   │   ├── package.xml        # ROS 包描述文件
    │   │   ├── move_panda.sh      # 机器人控制脚本
    │   │   ├── config/            # 配置文件目录
    │   │   │   └── panda_controller.yaml
    │   │   ├── launch/            # 启动文件目录
    │   │   │   └── panda_simulation.launch.py
    │   │   ├── urdf/              # URDF 模型目录
    │   │   │   ├── panda.urdf.xacro
    │   │   │   ├── panda_macro.urdf.xacro
    │   │   │   ├── panda_ros2control.xacro
    │   │   │   └── panda_transmission.xacro
    │   │   ├── meshes/            # 三维模型网格目录
    │   │   │   ├── collision/     # 碰撞模型
    │   │   │   └── visual/        # 视觉模型
    │   │   └── worlds/            # 仿真环境目录
    │   │       └── panda.world
    │   │
    │   └── panda_ros2_moveit2/    # MoveIt2 运动规划包
    │       ├── CMakeLists.txt     # CMake 构建配置
    │       ├── package.xml        # ROS 包描述文件
    │       ├── config/            # 配置文件目录
    │       │   ├── chomp_planning.yaml
    │       │   ├── kinematics.yaml
    │       │   ├── ompl_planning.yaml
    │       │   ├── panda.srdf
    │       │   ├── panda_controllers.yaml
    │       │   ├── panda_moveit2.rviz
    │       │   └── ros2_controllers.yaml
    │       └── launch/            # 启动文件目录
    │           └──panda.launch.py
    │
    └── panda_moveit_py/           # MoveIt Python 功能包
        ├── package.xml            # ROS 包描述文件
        ├── setup.cfg              # Python 安装配置
        ├── setup.py               # Python 安装脚本
        ├── panda_moveit_py/       # Python 模块目录
        │   ├── __init__.py
        │   ├── obstacle_manager.py         # 障碍物管理
        │   ├── pick_block.py               # 抓取方块功能
        │   ├── place_block.py              # 放置方块功能
        │   ├── pose_goal_client.py         # 路径规划
        │   └── pose_goal_client_home.py    # 路径规划（回位）
        ├── resource/              # 资源目录
        │   └── panda_moveit_py
        └── test/                  # 测试目录
            ├── test_copyright.py
            ├── test_flake8.py
            └── test_pep257.py




