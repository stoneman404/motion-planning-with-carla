# Motion Planning With Carla Simulator

[TOC]




## 1. Free Space MotionPlanner

hybrid a star is the main planner for open space planner. Note that this planner is not integrated with carla simulator.

### 1.1 Dependencies

- Ubuntu18.04
- ROS Melodic

### 1.2 Ros Graph

![](../graph_search_planner/figs/rosgraph.png)

### 1.3 Results

![](../graph_search_planner/figs/hybrid_astar_result1.png)

![](../graph_search_planner/figs/hybrid_astar_result2.png)

![](../graph_search_planner/figs/hybrid_astar_result3.png)


### 1.4 Future Works

- [ ] path smooth

- [ ] assign velocity profile

### 1.5 References

[1]  [Apollo Open Space MotionPlanner](https://github.com/ApolloAuto/apollo/tree/master/modules/planning) 

[2] [Hybrid A* Path MotionPlanner for the KTH Research Concept Vehicle](https://github.com/karlkurzer/path_planner)

## 2. On Reference Line MotionPlanner

### 2.1 Dependencies

- Ubuntu18.04
- ROS Melodic
- carla-simulator 0.9.9
- carla-ros-bridge

### 2.2 How To Use


### 2.3 Carla Route MotionPlanner
#### 2.3.1 Introduction
I modified the [carla_waypoint_publisher](https://github.com/carla-simulator/ros-bridge/tree/master/carla_waypoint_publisher) in [carla-ros-bridge](https://github.com/carla-simulator/ros-bridge)  to the route planner, which now is a ROS service.

**request**:

[RouteRequst](./planning_srvs/srv/Route.srv): provided the start pose and end pose

**response**:

[RouteResponse](./planning_srvs/srv/Route.srv): as long as received the RouteRequest, the planner will calculate the route from the start position to end position, the response is a list of [WayPoint](./planning_msgs/msg/WayPoint.msg).

this planner also provided another two services, which can be used to get waypoint by actor id and location, please refer to [carla_waypoint_publisher](https://github.com/carla-simulator/ros-bridge/tree/master/carla_waypoint_publisher).

#### 2.3.2 Methods


#### 2.3.3 References
1. [carla-ros-bridge](https://github.com/carla-simulator/ros-bridge)

2. [carla-simulator PythonAPI](https://carla.readthedocs.io/en/latest/python_api)

3. [carla_navigation](https://github.com/carla-simulator/carla/tree/master/PythonAPI/carla/agents/navigation)

### 2.4 State Lattice MotionPlanner 
#### 2.4.1 Introduction

#### 2.4.2 Methods

#### 2.4.3 References

### 2.5 Frenet Lattice MotionPlanner
#### 2.5.1 Introduction

#### 2.5.2 Methods

#### 2.5.3 References

### 2.6 SSC MotionPlanner
#### 2.6.1 Introduction

#### 2.6.2 Methods

#### 2.6.3 References



