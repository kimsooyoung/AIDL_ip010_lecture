source /usr/share/gazebo/setup.sh

- [] imu를 달아야 sensor fusion, cartographer를 할 수 있음
- [] gazebo 실행 시 urdf 지우고 다시 로컬 빌드하는 작업 해야 함 - 문서화 시 참고 
- [] [ERROR] [bt_navigator-4]: process has died [pid 39218, exit code -8, cmd '/opt/ros/foxy/lib/nav2_bt_navigator/bt_navigator --ros-args -r __node:=bt_navigator --params-file /tmp/tmpcxt2rvfx -r /tf:=tf -r /tf_static:=tf_static']. => 우분투 데탑에서는 또 안그러네
- [] pure pursuit 매개변수 수정
    desired_linear_vel : 0.7 => 0.5
    min_theta_velocity_threshold : 0.1 => 0.001 => 도착을 못하는 것 같음 => 다시 0.1
    yaw_goal_tolerance : 3.0 => 0.25
    use_rotate_to_heading : 이건 물류 로봇이면 무조건 true여야 하겠다.
    max_angular_accel : 3.2 => 2.0 => 느리게 돌다가 갑자기 휙 돌길래 롤백 => 3.2
    ㄴ 이게 가속도여서 그러네 
    rotate_to_heading_min_angle 전까지는 제자리 회전 + max_angular_accel의 절반? 속도로 주행 (3.2일 때 0.15 / 4.0일 때 0.2 / 5.0일 때 0.25)
    
    좀 더 타이트하게 직각직각으로 가게 하고프면
    rotate_to_heading_min_angle를 많이 줄이면 된다.
- [] 

# 처음부터 다시 세팅 하면서 로그

- ROS 2 설치

- apt install

sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  openssh-server \
  wget

sudo apt update && sudo apt install -y \
  gedit \
  ros-foxy-xacro \
  ros-foxy-joint-state-publisher \
  ros-foxy-joint-state-publisher-gui \
  ros-foxy-gazebo-ros-pkgs \
  ros-foxy-rqt-robot-steering \
  ros-foxy-robot-localization \
  ros-foxy-slam-toolbox \
  ros-foxy-nav2-* 


- xlaunch 설정

export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0 
export LIBGL_ALWAYS_INDIRECT=0

- vcs로 패키지 가져오기

cd ~/ros2_ws
wget https://raw.githubusercontent.com/kimsooyoung/AIDL_ip010_lecture/main/ip010.repos
vcs import src < ip010.repos

- 빌드

gazebo example



cmake 주석 해제 => .urdf 제거 => 빌드 => rosfoxy => 주석 해제 => 빌드
source /usr/share/gazebo/setup.sh

SF example

$ ros2 run gazebo_utils odom_utility_tools
[INFO] [1653984161.621405561] [reset_model_client]: ==== Entity State Service Client Ready ====


```
cd AIDL_ip010_lecture
./setup_scripts.sh
vcs import ../ < deps.repos

cd ~/ros2_ws
cbp ip010_description && rosfoxy
cbp ip010_gazebo && rosfoxy
cbp ip010_slam && rosfoxy
cbp ip010_slam && rosfoxy
cbp ip010_nav && rosfoxy
cbp gazebo_utils && rosfoxy

cbp aws_robomaker_small_warehouse_world && rosfoxy
```

slam

```
ros2 launch ip010_gazebo factory_world.launch.py
ros2 launch ip010_slam gazebo_slam_toolbox.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```