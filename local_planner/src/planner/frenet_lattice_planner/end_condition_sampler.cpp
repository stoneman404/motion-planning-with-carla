#include "planner/frenet_lattice_planner/end_condition_sampler.hpp"

#include <utility>

namespace planning {
using State = std::array<double, 3>;
using EndCondition = std::pair<std::array<double, 3>, double>;
EndConditionSampler::EndConditionSampler(const std::array<double, 3> &init_s,
                                         const std::array<double, 3> &init_d,
                                         std::shared_ptr<STGraph> ptr_st_graph)
    : init_s_(init_s),
      init_d_(init_d),
      ptr_st_graph_(std::move(ptr_st_graph)) {}

std::vector<EndCondition> EndConditionSampler::SampleLonEndConditionForStopping(
    const double ref_stop_point) const {
  constexpr size_t time_samples_num = 9;
  std::array<double, time_samples_num> time_samples{};
  for (size_t i = 1; i < time_samples_num; ++i) {
    auto ratio = static_cast<double>(i) / static_cast<double>(time_samples_num - 1);
    time_samples[i] = PlanningConfig::Instance().max_lookahead_time() * ratio;
  }
  time_samples[0] = PlanningConfig::Instance().min_lookahead_time();
  std::vector<EndCondition> end_s_conditions;
  for (const auto &time : time_samples) {
    State end_s = {std::max(init_s_[0], ref_stop_point), 0.0, 0.0};
    end_s_conditions.emplace_back(end_s, time);
  }
  return end_s_conditions;
}

std::vector<EndCondition> EndConditionSampler::SampleLatEndCondition() {
  std::vector<EndCondition> end_d_conditions;
  std::array<double, 3> end_d_candidates = {0, -0.5, 0.5};
  std::array<double, 4> end_s_candidates = {10.0, 20.0, 40.0, 60.0};
  for (const auto &s : end_s_candidates) {
    for (const auto &d : end_d_candidates) {
      State end_d_state = {d, 0.0, 0.0};
      end_d_conditions.emplace_back(end_d_state, s);
    }
  }
  return end_d_conditions;
}
std::vector<EndCondition> EndConditionSampler::SampleLonEndConditionForCruising(const double ref_target_vel) const {
  constexpr int vel_samples_num = 10;  // put  into PlanningConfig
  constexpr int time_samples_num = 9; // put into PlanningConfig
  constexpr double vel_interval_step = 0.2; // put int PlanningConfig
  std::array<double, time_samples_num> time_samples{};
  for (size_t i = 1; i < time_samples_num; ++i) {
    auto ratio = static_cast<double>(i) / static_cast<double>(time_samples_num - 1);
    time_samples[i] = PlanningConfig::Instance().max_lookahead_time() * ratio;
  }
  time_samples[0] = PlanningConfig::Instance().min_lookahead_time();
  std::vector<EndCondition> end_s_conditions;
  for (const auto &time : time_samples) {
    double v_upper = std::min(VUpper(time), ref_target_vel);
    double v_lower = VLower(time);
    State lower_end_s = {0.0, v_lower, 0.0};
    end_s_conditions.emplace_back(lower_end_s, time);
    State upper_end_s = {0.0, v_upper, 0.0};
    end_s_conditions.emplace_back(upper_end_s, time);

    double vel_gap = v_upper - v_lower;
    size_t vel_intervals_num =
        std::min(static_cast<size_t>(vel_samples_num - 2),
                 static_cast<size_t>(vel_gap / vel_interval_step));
    if (vel_intervals_num > 0) {
      double vel_ratio = vel_gap / static_cast<double>(vel_intervals_num);
      for (size_t i = 1; i <= vel_intervals_num; ++i) {
        State end_s = {0.0, v_lower + vel_ratio * static_cast<double>(i), 0.0};
        end_s_conditions.emplace_back(end_s, time);
      }
    }
  }
  return end_s_conditions;
}

double EndConditionSampler::VUpper(double t) const {
  return init_s_[1] + PlanningConfig::Instance().max_acc() * t;
}

double EndConditionSampler::VLower(double t) const {
  double t_at_zero_speed = init_s_[1] / PlanningConfig::Instance().max_acc();
  return t < t_at_zero_speed ?
         init_s_[1] - PlanningConfig::Instance().max_acc() * t :
         0.0;
}

double EndConditionSampler::SUpper(double t) const {
  return init_s_[0] + init_s_[1] * t +
      0.5 * PlanningConfig::Instance().max_acc() * t * t;
}

double EndConditionSampler::SLower(double t) const {
  double t_at_zero_speed = init_s_[1] / PlanningConfig::Instance().max_acc();
  double s_at_zero_speed = init_s_[0] + init_s_[1] * init_s_[1] / (2.0 * PlanningConfig::Instance().max_acc());
  return t < t_at_zero_speed ? init_s_[0] + init_s_[1] * t - PlanningConfig::Instance().max_acc() * t * t
                             : s_at_zero_speed;
}
std::vector<EndCondition> EndConditionSampler::SampleLonEndConditionWithObstacles() const {
  std::vector<EndCondition> end_s_conditions;

}

std::vector<std::pair<STPoint, double>> EndConditionSampler::OvertakeSamplePoints(int obstacle_id) const {
  std::vector<std::pair<STPoint, double>> sample_points{};
  std::vector<STPoint> overtake_st_points = ptr_st_graph_->GetObstacleSurroundingPoints(
      obstacle_id, 1e-3, PlanningConfig::Instance().delta_t());

  return sample_points;
}
std::vector<std::pair<STPoint, double>> EndConditionSampler::FollowingSamplePoints(int obstacle_id) const {
  std::vector<std::pair<STPoint, double>> sample_points{};

  return sample_points;
}

}
