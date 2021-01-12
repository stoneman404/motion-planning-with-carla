#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_BEHAVIOUR_CONFIGS_BEHAVIOUR_CONFIGS_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_BEHAVIOUR_CONFIGS_BEHAVIOUR_CONFIGS_HPP_
struct IDMParams {
  IDMParams()
      : desired_velocity(0.0),
        safe_time_headway(0.0),
        max_acc(0.0), max_decel(0.0),
        acc_exponent(4), s0(0.0), s1(0.0),
        leading_vehicle_length(0.0) {}
  IDMParams(double v, double t,
            double acc, double decel,
            int exp, double jam_distance_s0,
            double jam_distance_s1,
            double leading_length) : desired_velocity(v),
                                     safe_time_headway(t),
                                     max_acc(acc),
                                     max_decel(decel),
                                     acc_exponent(exp),
                                     s0(jam_distance_s0),
                                     s1(jam_distance_s1),
                                     leading_vehicle_length(leading_length) {}

  void PrintParams() const {
    std::cout << "=================IDM Params=====================" << std::endl;
    std::cout << "desired_vel : " << desired_velocity << ", safe_time_headway: " << safe_time_headway << std::endl;
    std::cout << "max_acc : " << max_acc << ", max_decel: " << max_decel << ", acc_exponent: " << acc_exponent << std::endl;
    std::cout << "s0 : " << s0 << ", s1: " << s1 << ", leading_vehicle_length: " << leading_vehicle_length << std::endl;
  }
  double desired_velocity{0.0}; // v0
  double safe_time_headway{1.0}; // T
  double max_acc{2.0}; // a
  double max_decel{5.0}; // b
  double comfortable_decel{2.0};
  int acc_exponent{4}; // /delta
  double s0{2.0}; //jam_distance
  double s1{0.0}; // jam distance
  double leading_vehicle_length{5.0};

};

struct SimulationParams {
  IDMParams idm_params;
//  double default_lateral_approach_ratio = 0.995;
//  double cutting_in_lateral_approach_ratio = 0.95;
  double sim_horizon_{10.0};
  double sim_step_{0.25};
  double steer_control_gain = 1.5;
  double steer_control_min_lookahead_dist = 3.0;
  double steer_control_max_lookahead_dist = 50.0;
  double max_lat_acceleration_abs = 1.5;
  double max_lat_jerk_abs = 3.0;
  double max_curvature_abs = 0.33;
  double max_lon_acc_jerk = 5.0;
  double max_lon_brake_jerk = 5.0;
  double max_steer_angle_abs = 45.0 / 180.0 * M_PI;
  double max_steer_rate = 0.39;
};

//struct SimulateConfigs {
//  double desired_vel{8.333};
//  double max_lat_acc{0.8};
//  double sim_horizon_{10.0};
//  double sim_step_{0.25};
//  double safe_time_headway{0.5}; // T
//  double max_acc{1.75}; // a
//  double max_decel{0.8}; // b
//  int acc_exponet{4}; // /delta
//  double s0{2.0}; // jam distance
//  double s1{0.0}; // jam distance
//  double default_lat_approach_ratio = 0.995;
//  double cutting_in_lateral_approach_ratio = 0.95;
//};

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_BEHAVIOUR_CONFIGS_BEHAVIOUR_CONFIGS_HPP_
