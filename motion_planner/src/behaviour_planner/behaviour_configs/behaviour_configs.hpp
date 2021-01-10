//
// Created by ldh on 1/10/21.
//

#ifndef CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_BEHAVIOUR_CONFIGS_BEHAVIOUR_CONFIGS_HPP_
#define CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_BEHAVIOUR_CONFIGS_BEHAVIOUR_CONFIGS_HPP_
struct IDMParams {
  IDMParams()
      : desired_velocity(0.0),
        safe_time_headway(0.0),
        max_acc(0.0), max_decel(0.0),
        acc_exponent(0.0), s0(0.0), s1(0.0),
        leading_vehicle_length(0.0) {}
  IDMParams(double v, double t,
            double acc, double decel,
            double exp, double jam_distance_s0,
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
  double desired_velocity{}; // v0
  double safe_time_headway{}; // T
  double max_acc{}; // a
  double max_decel{}; // b
  double acc_exponent{}; // /delta
  double s0{}; //jam_distance
  double s1{}; // jam distance
  double leading_vehicle_length{};
};

struct SimulationParams {
  IDMParams idm_params;
  double default_lateral_approach_ratio = 0.995;
  double cutting_in_lateral_approach_ratio = 0.95;
};

#endif //CATKIN_WS_SRC_MOTION_PLANNING_WITH_CARLA_MOTION_PLANNER_SRC_BEHAVIOUR_PLANNER_BEHAVIOUR_CONFIGS_BEHAVIOUR_CONFIGS_HPP_
