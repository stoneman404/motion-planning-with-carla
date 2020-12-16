#!/usr/bin/env python
import math
import numpy as np
from vehicle_pid_controller import Controller2D
from tf.transformations import euler_from_quaternion
from planning_msgs.msg import Trajectory
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaEgoVehicleStatus
from nav_msgs.msg import Odometry


import rospy


class VehicleState():
    def __init__(self, carla_ego_vehicle_info, odometry):
        self._x = odometry.pose.pose.position.x
        self._y = odometry.pose.pose.position.y
        self._z = odometry.pose.pose.position.z
        _, _, self._theta = euler_from_quaternion(odometry.pose.pose.orientation)
        self._v =  math.sqrt(odometry.twist.twist.linear.x ** 2 +
                                        odometry.twist.twist.linear.y ** 2 +
                                        odometry.twist.twist.linear.z ** 2)

    def v(self):
        return self._v

    def state(self):
        return self._x, self._y, self._z

    def theta(self):
        return self._theta


class CarlaController():

    def __init__(self):
        """
        Constructor
        """
        self._route_assigned = False
        self._global_plan = None
        self._target_speed = None
        self._ego_odometry = None
        self._vehicle_info = None
        self._vehicle_state = None
        self._trajectory = None
        self._controller2d = None

        rospy.on_shutdown(self.on_shutdown)

        # wait for ego vehicle
        self._vehicle_info = rospy.wait_for_message(
            "/carla/{}/vehicle_info".format("ego_vehicle"), CarlaEgoVehicleInfo)
        self._vehicle_odometry_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format("ego_vehicle"), Odometry, self.odometry_update)

        self._trajectory_subscriber = rospy.Subscriber(
            "/motion_planner/published_trajectory", Trajectory, self.trajectory_update)

        self._vehicle_control_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_cmd".format("ego_vehicle"), CarlaEgoVehicleControl, queue_size=1)

    def odometry_update(self, new_odometry):
        self._ego_odometry = new_odometry
        if self._vehicle_info:
            self._vehicle_state = VehicleState(self._vehicle_info, self._ego_odometry)


    @staticmethod
    def emergency_stop():
        """
        Send an emergency stop command to the vehicle

            :return: control for braking
        """
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = False

        return control

    def on_shutdown(self):
        """
        callback on shutdown
        """
        rospy.loginfo("Shutting down, stopping ego vehicle...")
        self._vehicle_control_publisher.publish(self.emergency_stop())


    def trajectory_update(self, trajectory):
        """
        callback on new route
        """
        trajectory = Trajectory
        rospy.loginfo("New plan with {} waypoints received.".format(len(path.poses)))
        if trajectory.status == Trajectory.EMPTY or trajectory.status == Trajectory.EMERGENCYSTOP:
            self._vehicle_control_publisher.publish(self.emergency_stop())
            return
        self._trajectory = trajectory
        self._has_trajectory = False

    def run_step(self):
        """
        Execute one step of navigation.
        """
        control = CarlaEgoVehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 0.0
        control.hand_brake = False

        if not self._vehicle_state:
            rospy.loginfo("Waiting for ego vehicle...")
            return control

        if not self._has_trajectory and self._trajectory:
            rospy.loginfo("Assigning trajectory...")
            self._agent._local_planner.set_global_plan(  # pylint: disable=protected-access
                self._global_plan.poses)
            self._route_assigned = True
        else:
            control = self._agent.run_step(self._target_speed)

        return control

    def run(self):
        """

        Control loop

        :return:
        """

        while not rospy.is_shutdown():
            control = self.run_step()
            if control:
                control.steer = -control.steer
                self.vehicle_control_publisher.publish(control)


def main():
    """

    main function

    :return:
    """
    controller = CarlaController()
    try:
        controller.run()
    finally:
        del controller
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
