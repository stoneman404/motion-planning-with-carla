# Planning

## 1. Requirements

- Ubuntu 18.04
- ros melodic
- carla-simulator 0.9.9
- carla-ros-bridge

tips:
我不想用scenario_runner
速度limit 应当有交规和路况决定
launch 文件:
- carla_ad_demo.launch
- carla_spectator_camera.launch


odometry 和 vehicle status 都是再"/map"下
odometry 包含pose, /carla/objects 这个topic 本身包含的是ego_vehicle的信息，包括位置，加速度，角速度　以及shape信息

在　/carla/ego_vehicle/vehicle_info中包含了汽车的最大最大转向角，重心位置

traffic lights 包括了所有的红绿灯信息，id, 状态
