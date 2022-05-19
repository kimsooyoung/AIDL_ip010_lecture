# AIDL_ip010_lecture

```
colcon build --symlink-install --packages-select ip010_description
source ./install/local_setup.bash

ros2 launch ip010_description ip010_description.launch.py
```

<img width="886" alt="image" src="https://user-images.githubusercontent.com/12381733/164188762-98a6ba84-fdd2-4a0f-a6dc-c5c6abe946fb.png">

## Gazebo

* example 1 - empty world

```
# clone aws-robomaker-small-warehouse-world pkg for external models
cd <your-ws>/src
git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
colcon build --symlink-install --packages-select aws_robomaker_small_warehouse_world

colcon build --symlink-install --packages-select ip010_gazebo
source ./install/local_setup.bash

ros2 launch ip010_gazebo empty_world.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/169349414-fc2bece6-f1a3-47fb-837f-6b6f2ff16f3a.png)


* example 2 - factory world

```
ros2 launch ip010_gazebo factory_world.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/169349575-8e53d2c8-4635-4e9b-9056-1f0ba197cec8.png)

* example 3 - Small Village

```

```

## Odom utils & Sensor Fusion

* odom comparison

```
colcon build --symlink-install --packages-select gazebo_utils
source ./install/local_setup.bash

ros2 launch ip010_gazebo empty_world.launch.py open_rviz:=false
ros2 run gazebo_utils odom_utility_tools
```

Run `rqt` then spawn 4 rqt_plots.

![image](https://user-images.githubusercontent.com/12381733/169341841-ca9b6dde-5245-437d-a742-eeff2a458d60.png)

Then compare gt odom with odom topic's values.

* Sensor Fusion

```

```

## SLAM

* slam_toolbox

```
colcon build --symlink-install --packages-select ip010_slam
source ./install/local_setup.bash

ros2 launch ip010_gazebo factory_world.launch.py open_rviz:=false
ros2 launch ip010_slam gazebo_slam_toolbox.launch.py 
```

save map

```

```

![image](https://user-images.githubusercontent.com/12381733/169345139-15c5de3b-5117-446f-9dd1-b86b787071e6.png)

* cartographer

```

```

## Localization

```
ros2 launch ip010_gazebo factory_world.launch.py open_rviz:=false

ros2 launch ip010_amcl amcl.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/169355890-c182afdd-8921-4f9d-bc79-1c450ec8a139.png)

## Navigation

```
ros2 launch ip010_gazebo factory_world.launch.py open_rviz:=false

ros2 launch ip010_nav bringup_launch.py 

ros2 launch ip010_nav localization_launch.py use_sim_time:=true
ros2 launch ip010_nav navigation_launch.py use_sim_time:=true
ros2 launch ip010_nav rviz_view_launch.py use_sim_time:=true
```

## Obstable avoidance

## Swarming

## Parking

## (Option) Deep Learning Nodes

- /grond_truth_x/data vs /odom/pose/pose/position/x
- /grond_truth_y/data vs /odom/pose/pose/position/y

[] gif add



[] dependency check & rosdep setup
[] sensor fusion
[] localization
[] slam
[] navigation
[] AIDL_ip010_robot