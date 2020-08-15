# the id of this waypoint
int32 id
# the road id of this waypoint
int32 road_id
# the section id of this waypoint
int32 section_id
# the lane id of this waypoint
int32 lane_id
 # if this waypoint is in junction
bool is_junction
bool has_left_lane
bool has_right_lane
bool has_value
# the pose of this waypoint
geometry_msgs/Pose pose
# the lane width
float64 lane_width
float64 right_lane_width
float64 left_lane_width
# the lane change type of this waypoint,
# allowed to change LEFT, RIGTH, BOTH or NONE
LaneChangeType lane_change
# the lane type, eg. the driving lane
LaneType lane_type
# the left lane marks
LaneMarking left_lane_marking
# the right lane marks
LaneMarking right_lane_marking

# to get junction should make sure is_junction==true
Junction junction
CarlaRoadOption road_option
float64 s # s value of this the waypoint