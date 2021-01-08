#!/usr/bin/env python
import math
import sys
import threading
import glob
import os
import random

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import Color

import rospy
from agents.navigation.local_planner import RoadOption
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.global_route_planner import GlobalRoutePlanner
from carla_msgs.msg import CarlaWorldInfo
from carla_waypoint_types.srv import GetActorWaypointResponse, GetActorWaypoint
from carla_waypoint_types.srv import GetWaypointResponse, GetWaypoint
from planning_msgs.msg import WayPoint, LaneType, CarlaRoadOption, LaneChangeType
from planning_srvs.srv import RoutePlanService, RoutePlanServiceResponse, AgentRouteService, AgentRouteServiceResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from planning_msgs.msg import LaneArray, Lane
from route_planner import RandomRoutePlanner

WAYPOINT_DISTANCE = 2.0


class ClinetInterface(object):

    def __init__(self, carla_world):
        """
        Constructor
        """
        self._world = carla_world
        self._map = carla_world.get_map()
        self._ego_vehicle = None
        self._ego_vehicle_location = None
        # self.current_route_list = RouteList()
        self._on_tick = None
        self._goal = None
        self._role_name = rospy.get_param("~role_name", 'ego_vehicle')
        self._dao = GlobalRoutePlannerDAO(self._map, sampling_resolution=WAYPOINT_DISTANCE)
        self._random_route_planner = RandomRoutePlanner(self._dao)
        self._random_route_planner.setup()
        self._global_planner = GlobalRoutePlanner(self._dao)
        self._global_planner.setup()

        self._getWaypointService = rospy.Service(
            '/carla_waypoint_publisher/{}/get_waypoint'.format(self._role_name),
            GetWaypoint, self._get_waypoint)
        self._getActorWaypointService = rospy.Service(
            '/carla_waypoint_publisher/{}/get_actor_waypoint'.format(self._role_name),
            GetActorWaypoint, self._get_actor_waypoint)
        self._getRouteService = rospy.Service('/carla_client_interface/{}/get_route'.format(self._role_name),
                                              RoutePlanService, self._get_route)
        self._getAgentPotentialRoutesService = rospy.Service(
            '/carla_client_interface/{}/agent_potential_routes'.format(self._role_name),
            RoutePlanService, self._get_agent_potential_route_service)
        self._update_lock = threading.Lock()

        rospy.loginfo("Waiting for ego vehicle .....")

    #     self.on_tick = self.world.on_tick(self.find_ego_vehicle_actor)

    def destroy(self):
        """
        Destructor
        """
        self._ego_vehicle = None
        if self._on_tick:
            self._world.remove_on_tick(self._on_tick)

    def _get_waypoint(self, req):
        """
        Get the waypoint for a location
        """
        carla_position = carla.Location()
        carla_position.x = req.location.x
        carla_position.y = -req.location.y
        carla_position.z = req.location.z

        carla_waypoint = self._map.get_waypoint(carla_position)

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

    def _get_agent_potential_route_service(self, req):

        actor = self._world.get_actors().find(req.actor_id)
        response = AgentRouteServiceResponse()
        if actor:
            waypoint = self._map.get_waypoint(actor.get_location())
            lane_array = self._random_route_planner.get_potential_paths(waypoint.transform.location)

            if waypoint.lane_change == carla.LaneChange.Both or waypoint.lane_change == carla.LaneChange.Right:
                right_waypoint = waypoint.get_right_lane()
                if right_waypoint:
                    right_lane_array = self._random_route_planner.get_potential_paths(right_waypoint.transform.location)
                    lane_array = lane_array + right_lane_array

            if waypoint.lane_change == carla.LaneChange.Both or waypoint.lane_change == carla.LaneChange.Left:
                left_waypoint = waypoint.get_left_lane()
                if left_waypoint:
                    left_lane_array = self._random_route_planner.get_potential_paths(left_waypoint.transform.location)
                    lane_array = lane_array + left_lane_array
            response = AgentRouteServiceResponse()
            for lane in lane_array:
                route = []
                for waypoint in lane:
                    route.append([waypoint, RoadOption.LANEFOLLOW])
                response.lanes.append(self.route_to_planning_waypoint(lane))

        else:
            rospy.logwarn("get_actor_waypoint(): Actor {} not valid.".format(req.id))
        return response

    def _get_actor_waypoint(self, req):
        """
        Convenience method to get the waypoint for an actor
        """
        rospy.loginfo("get_actor_waypoint(): Get waypoint of actor {}".format(req.id))
        actor = self._world.get_actors().find(req.id)

        response = GetActorWaypointResponse()
        if actor:
            carla_waypoint = self._map.get_waypoint(actor.get_location())
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

    def route_to_planning_waypoint(self, route):

        lane = Lane()
        for wp in route:
            last_x = wp[0].transform.location.x
            last_y = wp[0].transform.location.y
            waypoint = WayPoint()
            waypoint.pose.position.x = wp[0].transform.location.x
            waypoint.pose.position.y = -wp[0].transform.location.y
            waypoint.pose.position.z = wp[0].transform.location.z
            quaternion = quaternion_from_euler(
                0, 0, -math.radians(wp[0].transform.rotation.yaw))
            waypoint.pose.orientation.x = quaternion[0]
            waypoint.pose.orientation.y = quaternion[1]
            waypoint.pose.orientation.z = quaternion[2]
            waypoint.pose.orientation.w = quaternion[3]
            waypoint.road_id = wp[0].lane_id
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
            lane.way_points.append(waypoint)
            # response.route.append(waypoint)

        if len(lane.way_points) == 0:
            rospy.logerr("the lane.way_points is empty")
        else:
            rospy.loginfo(
                "update the response, success, the waypoints in response.route is {}".format(len(lane.way_points)))
        return lane

    def _get_route(self, req):
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

        rospy.loginfo("Calculating route from x={}, y={}, z={} to x={}, y={}, z={}".format(
            carla_start.location.x,
            carla_start.location.y,
            carla_start.location.z,
            carla_goal.location.x,
            carla_goal.location.y,
            carla_goal.location.z))

        carl_start_waypoint = self._map.get_waypoint(carla_start.location)
        carla_goal_waypoint = random.choice(self._world.get_map().get_spawn_points())

        raw_route = self._random_route_planner.trace_route(carl_start_waypoint.transform.location,
                                                       carla_goal_waypoint.location)

        extend_route = list()
        waypoints = carl_start_waypoint.previous_until_lane_start(2.0)
        option = RoadOption.LANEFOLLOW
        for wp in waypoints:
            extend_route.append([wp, option])
        extend_route.reverse()
        #
        raw_route = extend_route + raw_route
        last_wp = raw_route[0][0]
        route = list(list())
        route.append([last_wp, raw_route[0][1]])
        for i in range(len(raw_route)):
            if i == 0:
                continue
            wp = raw_route[i][0]
            dist = wp.transform.location.distance(last_wp.transform.location)
            if dist < 0.5:
                continue
            last_wp = wp
            route.append(raw_route[i])


        right_lane = list()
        left_lane = list()
        for wp in route:
            waypoint = wp[0]
            if (
                    waypoint.lane_change == carla.LaneChange.Both or waypoint.lane_change == carla.LaneChange.Right) and waypoint.get_right_lane():
                right_lane.append([waypoint.get_right_lane(), wp[1]])
            if (
                    waypoint.lane_change == carla.LaneChange.Both or waypoint.lane_change == carla.LaneChange.Left) and waypoint.get_left_lane():
                left_lane.append([waypoint.get_left_lane(), wp[1]])
        #
        # response = RoutePlanServiceResponse()
        # count = 0
        # road_option = RoadOption.LANEFOLLOW
        #
        # waypoints = list(list())
        # left_waypoints = list(list())
        # right_waypoints = list(list())
        # front_waypoints = carl_start_waypoint.previous_until_lane_start(distance=2.0)
        # front_waypoints.reverse()
        # for wp in front_waypoints:
        #     waypoints.append([wp, road_option])
        #
        # if carl_start_waypoint.get_right_lane() and (
        #         carl_start_waypoint.lane_change == carla.LaneChange.Right or carl_start_waypoint.lane_change == carla.LaneChange.Both):
        #     right_front_waypoints = carl_start_waypoint.get_right_lane().previous_until_lane_start(distance=2.0)
        #     right_front_waypoints.reverse()
        #
        #     for wp in right_front_waypoints:
        #         right_waypoints.append([wp, road_option])
        # right_waypoints.append([carl_start_waypoint.get_right_lane(), road_option])
        # if carl_start_waypoint.get_left_lane() and (
        #         carl_start_waypoint.lane_change == carla.LaneChange.Left or carl_start_waypoint.lane_change == carla.LaneChange.Both):
        #     left_front_waypoints = carl_start_waypoint.get_left_lane().previous_until_lane_start(distance=2.0)
        #     left_front_waypoints.reverse()
        #
        #     for wp in left_front_waypoints:
        #         left_waypoints.append([wp, road_option])
        # left_waypoints.append([carl_start_waypoint.get_left_lane(), road_option])
        # last_waypoint = carl_start_waypoint
        # waypoints.append([carl_start_waypoint, road_option])
        # while True:
        #     if count < 150:
        #         nexts = list(last_waypoint.next(distance=2.0))
        #         if not nexts:
        #             raise RuntimeError("No more waypoints")
        #         w = random.choice(nexts)
        #         waypoints.append([w, road_option])
        #         if w.get_right_lane() and (
        #                 w.lane_change == carla.LaneChange.Right or w.lane_change == carla.LaneChange.Both):
        #             right_waypoints.append([w.get_right_lane(), road_option])
        #         if w.get_left_lane() and (
        #                 w.lane_change == carla.LaneChange.Left or w.lane_change == carla.LaneChange.Both):
        #             left_waypoints.append([w.get_left_lane(), road_option])
        #         count += 1
        #         last_waypoint = w
        #     else:
        #         break
        for wp in route:
            self._world.debug.draw_point(wp[0].transform.location + carla.Location(z=1), color = Color(100, 255, 1), life_time=120)
        for wp in left_lane:
            self._world.debug.draw_point(wp[0].transform.location + carla.Location(z=1), color = Color(100, 100, 1), life_time=120)
        for wp in right_lane:
            self._world.debug.draw_point(wp[0].transform.location + carla.Location(z=1), color = Color(255, 0, 1), life_time=120)
        response = RoutePlanServiceResponse()
        if len(route) > 0:
            route.pop(0)
        if len(left_lane) > 0:
            left_lane.pop(0)
        if len(right_lane) > 0:
            right_lane.pop(0)
        response.right_lane = self.route_to_planning_waypoint(right_lane)
        response.left_lane = self.route_to_planning_waypoint(left_lane)
        response.route = self.route_to_planning_waypoint(route)
        rospy.loginfo("the route length is {}".format(len(response.route.way_points)))
        for wp in response.route.way_points:
            print("wp : x:{}, y:{}, lane_id:{}, road_id:{}".format(wp.pose.position.x, wp.pose.position.y, wp.lane_id,
                                                                   wp.road_id))
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
        carlaInterface = ClinetInterface(carla_world)
        rospy.spin()
        carlaInterface.destroy()
        del carlaInterface
        del carla_world
        del carla_client

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
