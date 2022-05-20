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
