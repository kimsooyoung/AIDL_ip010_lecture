# AIDL_ip010_lecture

* Autonomous driving simulation based on ROS 2's Nav2 and AIDL's IP010

## Prerequisite

* ROS 2 (> Foxy)
* Gazebo (Not a Ignition Gazebo, Gazebo 11 maybe)
* Dependency Packages

## Prepare Dependencies

```
# Clone this Repo
git clone

# Install all ROS 2 depends through shell scripts
cd AIDL_ip010_lecture
./setup_scripts.sh
vcs import ../ < deps.repos

# Build Packages
cd ~/<your-ws>

cbp ip010_amcl && rosfoxy
cbp ip010_description && rosfoxy
cbp ip010_gazebo && rosfoxy
cbp ip010_nav && rosfoxy
cbp ip010_slam && rosfoxy
cbp gazebo_utils && rosfoxy
cbp aws_robomaker_small_warehouse_world && rosfoxy
cbp nav2_rosdevday_2021 && rosfoxy
```

# Examples

* IP010 description

```
ros2 launch ip010_description ip010_description.launch.py
```

<p align="center">
    <img src="https://user-images.githubusercontent.com/12381733/164188762-98a6ba84-fdd2-4a0f-a6dc-c5c6abe946fb.png" height="250">
</p>

* Gazebo example 1 - empty world

```
ros2 launch ip010_gazebo empty_world.launch.py
```

<p align="center">
    <img src="https://user-images.githubusercontent.com/12381733/169349414-fc2bece6-f1a3-47fb-837f-6b6f2ff16f3a.png" height="250">
</p>


* Gazebo example 2 - AWS Warehouse world

```
ros2 launch ip010_gazebo factory_world.launch.py
```

<p align="center">
    <img src="https://user-images.githubusercontent.com/12381733/169349575-8e53d2c8-4635-4e9b-9056-1f0ba197cec8.png" height="250">
</p>


---

## Nav2 related Example

* slam_toolbox

```
ros2 launch ip010_gazebo factory_world.launch.py open_rviz:=false
ros2 launch ip010_slam gazebo_slam_toolbox.launch.py 
```

<p align="center">
    <img src="https://user-images.githubusercontent.com/12381733/169345139-15c5de3b-5117-446f-9dd1-b86b787071e6.png" height="250">
</p>


* AMCL - Localization

```
ros2 launch ip010_gazebo factory_world.launch.py open_rviz:=false
ros2 launch ip010_amcl amcl.launch.py
```

<p align="center">
    <img src="https://user-images.githubusercontent.com/12381733/169355890-c182afdd-8921-4f9d-bc79-1c450ec8a139.png" height="250">
</p>

* Navigation - pure pursuit, dwb andvarious parameter changed examples exists

```
ros2 launch ip010_gazebo factory_world.launch.py open_rviz:=false

ros2 launch ip010_nav bringup_launch.py 

ros2 launch ip010_nav localization_launch.py use_sim_time:=true
ros2 launch ip010_nav navigation_launch.py use_sim_time:=true
ros2 launch ip010_nav rviz_view_launch.py use_sim_time:=true
```

<p align="center">
    <img src="https://user-images.githubusercontent.com/12381733/181112851-626d1143-6d70-4105-8768-03c7e8235417.png" height="250">
</p>