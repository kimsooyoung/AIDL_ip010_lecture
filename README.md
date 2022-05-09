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
colcon build --symlink-install --packages-select ip010_description
source ./install/local_setup.bash

ros2 launch ip010_gazebo empty_world.launch.py
```

* example 2 - factory world

```
ros2 launch ip010_gazebo factory_world.launch.py
```

* example 3 - Small Village

```

```

## Odom utils & Sensor Fusion

* odom comparison

```
colcon build --symlink-install --packages-select gazebo_utils
source ./install/local_setup.bash

ros2 launch ip010_gazebo empty_world.launch.py

ros2 run gazebo_utils odom_utility_tools
```

Execute `rqt` and spawn 4 rqt_plots.
Then compare BTW gt odom and odom topic's odom

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

* cartographer

```

```

## Localization

## Navigation


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