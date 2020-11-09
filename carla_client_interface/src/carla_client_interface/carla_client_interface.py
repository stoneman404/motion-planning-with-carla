#!/usr/bin/env python
import math
import sys
import threading

import carla
import numpy as np
import rospy
import tf
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.local_planner import RoadOption
from carla_msgs.msg import CarlaWorldInfo
from carla_waypoint_types.srv import GetActorWaypointResponse, GetActorWaypoint
from carla_waypoint_types.srv import GetWaypointResponse, GetWaypoint
from planning_msgs.msg import WayPoint, LaneType, CarlaRoadOption, LaneChangeType
from planning_srvs.srv import Route, RouteResponse
from tf.transformations import euler_from_quaternion


class ClinetInterface(object):
    WAYPOINT_DISTANCE = 2.0

    def __init__(self, carla_world):
        """
        Constructor
        """
        self.world = carla_world
        self.map = carla_world.get_map()
        self.ego_vehicle = None
        self.ego_vehicle_location = None
        self.on_tick = None
        self.goal = None
        self.role_name = rospy.get_param("~role_name", 'ego_vehicle')
        # self.waypoint_publisher = rospy.Publisher(
        #     '/carla/{}/waypoints'.format(self.role_name), Path, queue_size=1, latch=True)

        # initialize ros services
        self.getWaypointService = rospy.Service(
            '/carla_waypoint_publisher/{}/get_waypoint'.format(self.role_name),
            GetWaypoint, self.get_waypoint)
        self.getActorWaypointService = rospy.Service(
            '/carla_waypoint_publisher/{}/get_actor_waypoint'.format(self.role_name),
            GetActorWaypoint, self.get_actor_waypoint)
        self.getRouteService = rospy.Service('/carla_client_interface/{}/get_route'.format(self.role_name),
                                             Route, self.get_route)
        self._update_lock = threading.Lock()


    def destroy(self):
        """
        Destructor
        """
        self.ego_vehicle = None
        if self.on_tick:
            self.world.remove_on_tick(self.on_tick)

    def get_waypoint(self, req):
        """
        Get the waypoint for a location
        """
        carla_position = carla.Location()
        carla_position.x = req.location.x
        carla_position.y = -req.location.y
        carla_position.z = req.location.z

        carla_waypoint = self.map.get_waypoint(carla_position)

        response = GetWaypointResponse()

        response.waypoint.pose.position.x = carla_waypoint.transform.location.x
        response.waypoint.pose.position.y = -carla_waypoint.transform.location.y
        response.waypoint.pose.position.z = carla_waypoint.transform.location.z
        response.waypoint.is_junction = carla_waypoint.is_junction
        response.waypoint.road_id = carla_waypoint.road_id
        response.waypoint.section_id = carla_waypoint.section_id
        response.waypoint.lane_id = carla_waypoint.lane_id
        rospy.logwarn("Get waypoint {}".format(response.waypoint.pose.position))
        return response

    def get_actor_waypoint(self, req):
        """
        Convenience method to get the waypoint for an actor
        """
        rospy.loginfo("get_actor_waypoint(): Get waypoint of actor {}".format(req.id))
        actor = self.world.get_actors().find(req.id)

        response = GetActorWaypointResponse()
        if actor:
            carla_waypoint = self.map.get_waypoint(actor.get_location())
            response.waypoint.pose.position.x = carla_waypoint.transform.location.x
            response.waypoint.pose.position.y = -carla_waypoint.transform.location.y
            response.waypoint.pose.position.z = carla_waypoint.transform.location.z
            response.waypoint.is_junction = carla_waypoint.is_junction
            response.waypoint.road_id = carla_waypoint.road_id
            response.waypoint.section_id = carla_waypoint.section_id
            response.waypoint.lane_id = carla_waypoint.lane_id
        else:
            rospy.logwarn("get_actor_waypoint(): Actor {} not valid.".format(req.id))
        return response

    def get_route(self, req):
        """
        :return:
        """
        rospy.loginfo("has received request, handling request")
        # retrieve goal pose
        carla_goal = carla.Transform()
        carla_goal.location.x = req.end_pose.position.x
        carla_goal.location.y = -req.end_pose.position.y
        carla_goal.location.z = req.end_pose.position.z + 2
        end_quaternion = (
            req.end_pose.orientation.x,
            req.end_pose.orientation.y,
            req.end_pose.orientation.z,
            req.end_pose.orientation.w
        )
        _, _, end_yaw = euler_from_quaternion(end_quaternion)
        carla_goal.rotation.yaw = -math.degrees(end_yaw)
        # retrieve init pose
        carla_start = carla.Transform()
        carla_start.location.x = req.start_pose.position.x
        carla_start.location.y = -req.start_pose.position.y
        carla_start.location.z = req.start_pose.position.z + 2
        start_quaternion = (
            req.start_pose.orientation.x,
            req.start_pose.orientation.y,
            req.start_pose.orientation.z,
            req.start_pose.orientation.w
        )
        _, _, start_yaw = euler_from_quaternion(start_quaternion)
        carla_start.rotation.yaw = -math.degrees(start_yaw)

        rospy.loginfo("Calculating route to x={}, y={}, z={}".format(
            carla_goal.location.x,
            carla_goal.location.y,
            carla_goal.location.z))
        dao = GlobalRoutePlannerDAO(self.world.get_map(), sampling_resolution=1)
        grp = GlobalRoutePlanner(dao)
        grp.setup()
        # route = grp.trace_route(carla.Location(carla_start.location.x,
        #                                        carla_start.location.y,
        #                                        carla_start.location.y),
        #                         carla.Location(carla_goal.location.x,
        #                                        carla_goal.location.y,
        #                                        carla_goal.location.z))
        route = list(list())
        carl_start_waypoint = self.map.get_waypoint(carla_start.location)
        # waypoints = carl_start_waypoint.next_until_lane_end(2)
        waypoints = []
        last_way_point = carl_start_waypoint
        s = 0.0
        while s < 800:
            index = np.random.randint(0, len(last_way_point.next(2.0)))

            waypoint = last_way_point.next(2.0)[index]
            waypoints.append(waypoint)
            last_way_point = waypoint
            s += 2.0
        rospy.loginfo("the waypoints.size is {}".format(len(waypoints)))
        option = RoadOption.LANEFOLLOW
        for wp in waypoints:
            route.append([wp, option])
        # get prev waypoint as start

        waypoint = route[0][0].previous(distance=2.0)[-1]
        route.insert(0, [waypoint, option])
        response = RouteResponse()
        # waypoint
        last_x = float("inf")
        last_y = float("inf")
        for wp in route:
            # if math.fabs(wp[0].transform.location.x - last_x) < 0.2 and math.fabs(
            #         wp[0].transform.location.y - last_y) < 0.2:
            #     last_x = wp[0].transform.location.x
            #     last_y = wp[0].transform.location.y
            #     continue
            last_x = wp[0].transform.location.x
            last_y = wp[0].transform.location.y
            waypoint = WayPoint()
            waypoint.pose.position.x = wp[0].transform.location.x
            waypoint.pose.position.y = -wp[0].transform.location.y
            waypoint.pose.position.z = wp[0].transform.location.z
            quaternion = tf.transformations.quaternion_from_euler(
                0, 0, -math.radians(wp[0].transform.rotation.yaw))
            waypoint.pose.orientation.x = quaternion[0]
            waypoint.pose.orientation.y = quaternion[1]
            waypoint.pose.orientation.z = quaternion[2]
            waypoint.pose.orientation.w = quaternion[3]
            # print(wp[0].id)
            waypoint.road_id = wp[0].lane_id
            # print(wp[0].lane_id)
            waypoint.section_id = wp[0].section_id
            waypoint.lane_id = wp[0].lane_id
            waypoint.is_junction = wp[0].is_junction
            waypoint.has_value = True
            left_lane = wp[0].get_left_lane()
            right_lane = wp[0].get_right_lane()
            if left_lane is None:
                waypoint.left_lane_width = -1.0
                waypoint.has_left_lane = False
            else:
                waypoint.has_left_lane = True
                waypoint.left_lane_width = wp[0].get_left_lane().lane_width
            if right_lane is None:
                waypoint.right_lane_width = -1.0
                waypoint.has_right_lane = False
            else:
                waypoint.right_lane_width = wp[0].get_right_lane().lane_width
                waypoint.has_right_lane = True
            waypoint.lane_width = wp[0].lane_width
            waypoint.lane_type.type = self.set_lane_type(wp[0])

            lane_change_type = wp[0].lane_change
            if lane_change_type == carla.LaneChange.NONE:
                waypoint.lane_change.type = LaneChangeType.FORWARD
            elif lane_change_type == carla.LaneChange.Both:
                waypoint.lane_change.type = LaneChangeType.BOTH
            elif lane_change_type == carla.LaneChange.Left:
                waypoint.lane_change.type = LaneChangeType.LEFT
            elif lane_change_type == carla.LaneChange.Right:
                waypoint.lane_change.type = LaneChangeType.RIGHT

            waypoint.road_option.option = self.set_road_option(wp)
            # print(waypoint.road_option)
            if waypoint.is_junction:
                junction = wp[0].get_junction()
                waypoint.junction.id = junction.id
                waypoint.junction.bounding_box.location.x = junction.bounding_box.location.x
                waypoint.junction.bounding_box.location.y = -junction.bounding_box.location.y
                waypoint.junction.bounding_box.location.z = junction.bounding_box.location.z
                waypoint.junction.bounding_box.extent.x = junction.bounding_box.extent.x
                waypoint.junction.bounding_box.extent.y = junction.bounding_box.extent.y
                waypoint.junction.bounding_box.extent.z = junction.bounding_box.extent.z

            waypoint.s = wp[0].s

            response.route.append(waypoint)

        if len(response.route) == 0:
            rospy.logerr("the response.route is empty")
        else:
            rospy.loginfo(
                "update the response, success, the waypoints in response.route is {}".format(len(response.route)))

            # for wp in response.route:

            #     print("waypoint x: {}, y:{}".format(wp.pose.position.x,
            #                                         wp.pose.position.y))

        return response

    def set_road_option(self, wp):
        waypoint_road_option = CarlaRoadOption()
        road_option = wp[1]
        if road_option == RoadOption.LEFT:
            waypoint_road_option = CarlaRoadOption.LEFT
        elif road_option == RoadOption.RIGHT:
            waypoint_road_option = CarlaRoadOption.RIGHT
        elif road_option == RoadOption.CHANGELANELEFT:
            waypoint_road_option = CarlaRoadOption.CHANGELANELEFT
        elif road_option == RoadOption.CHANGELANERIGHT:
            waypoint_road_option = CarlaRoadOption.CHANGELANERIGHT
        elif road_option == RoadOption.LANEFOLLOW:
            waypoint_road_option = CarlaRoadOption.LANEFOLLOW
        elif road_option == RoadOption.STRAIGHT:
            waypoint_road_option = CarlaRoadOption.STRAIGHT
        elif road_option == RoadOption.VOID:
            waypoint_road_option = CarlaRoadOption.VOID

        return waypoint_road_option

    def set_lane_type(self, wp):

        lane_type = wp.lane_type
        waypoint_lane_type = LaneType()
        if lane_type == carla.LaneType.Bidirectional:
            waypoint_lane_type = LaneType.BIDIRECTIONAL
        elif lane_type == carla.LaneType.Restricted:
            waypoint_lane_type = LaneType.RESTRICTED
        elif lane_type == carla.LaneType.Any:
            waypoint_lane_type = LaneType.ANY
        elif lane_type == carla.LaneType.Biking:
            waypoint_lane_type = LaneType.BIKING
        elif lane_type == carla.LaneType.Border:
            waypoint_lane_type = LaneType.BORDER
        elif lane_type == carla.LaneType.Driving:
            waypoint_lane_type = LaneType.DRIVING
        elif lane_type == carla.LaneType.Entry:
            waypoint_lane_type = LaneType.ENTRY
        elif lane_type == carla.LaneType.Exit:
            waypoint_lane_type = LaneType.EXIT
        elif lane_type == carla.LaneType.Median:
            waypoint_lane_type = LaneType.MEDIAN
        elif lane_type == carla.LaneType.NONE:
            waypoint_lane_type = LaneType.NONE
        elif lane_type == carla.LaneType.OffRamp:
            waypoint_lane_type = LaneType.OFFRAMP
        elif lane_type == carla.LaneType.OnRamp:
            waypoint_lane_type = LaneType.ONRAMP
        elif lane_type == carla.LaneType.Parking:
            waypoint_lane_type = LaneType.PARKING
        elif lane_type == carla.LaneType.Rail:
            waypoint_lane_type = LaneType.RAIL
        elif lane_type == carla.LaneType.RoadWorks:
            waypoint_lane_type = LaneType.ROADWORKS
        elif lane_type == carla.LaneType.Shoulder:
            waypoint_lane_type = LaneType.SHOULDER
        elif lane_type == carla.LaneType.Sidewalk:
            waypoint_lane_type = LaneType.SIDEWALK
        elif lane_type == carla.LaneType.Special1:
            waypoint_lane_type = LaneType.SPECIAL1
        elif lane_type == carla.LaneType.Special2:
            waypoint_lane_type = LaneType.SPECIAL2
        elif lane_type == carla.LaneType.Special3:
            waypoint_lane_type = LaneType.SPECIAL3
        elif lane_type == carla.LaneType.Stop:
            waypoint_lane_type = LaneType.STOP
        elif lane_type == carla.LaneType.Tram:
            waypoint_lane_type = LaneType.TRAM

        return waypoint_lane_type


def main():
    """
    main function
    """
    try:
        rospy.init_node("carla_client_interface", anonymous=True)
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("Error while waiting for world info!")
        sys.exit(1)

    host = rospy.get_param("/carla/host", "127.0.0.1")
    port = rospy.get_param("/carla/port", 2000)
    timeout = rospy.get_param("/carla/timeout", 2)

    rospy.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
        host=host, port=port))

    try:
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        carla_world = carla_client.get_world()

        rospy.loginfo("Connected to Carla.")

        waypointConverter = ClinetInterface(carla_world)

        rospy.spin()
        waypointConverter.destroy()
        del waypointConverter
        del carla_world
        del carla_client

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
