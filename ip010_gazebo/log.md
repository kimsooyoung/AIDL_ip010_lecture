

# TODO

* odom util로 측정
![image](https://user-images.githubusercontent.com/12381733/166242928-ec2a21ad-f8ed-4ee4-9459-e8df9a3a6298.png)

* 속도 제어 실제로 되는지

twist가 개판이네...

![image](https://user-images.githubusercontent.com/12381733/166243264-3a37be4b-6b9e-4659-8d13-a2bdabff5b66.png)


front_ballcaster에 mu를 줘야지 base_link에 주고 있었음 ㅋㅋ
```
  <gazebo reference="front_ballcaster">
    <material>${body_color}</material>
    <mu1>0.01</mu1>
    <mu2>0.01</mu2>
    <selfCollide>true</selfCollide>
    <gravity>true</gravity>
  </gazebo>
```

해결!

지 혼자 움직이는 것도 해결
mu 0.01 => 0.1로 변경

```
ros2 run rqt_image_view rqt_image_view
```

새로운 환경에서는 ip010_description_new.urdf를 지운 다음 다시 빌드해야 한다.

* AWS Factory Gazebo


```
git clone -b foxy-devel https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
cbp aws_robomaker_small_warehouse_world && rosfoxy
source /usr/share/gazebo/setup.sh

ros2 launch fusionbot_gazebo aws_factory.launch.py
```

TODO

naming change => ip010_description_new.urdf
factory world가 새로운 환경에서도 잘 되는지


