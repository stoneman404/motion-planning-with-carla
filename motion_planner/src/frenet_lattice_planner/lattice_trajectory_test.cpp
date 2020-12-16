#include <gtest/gtest.h>
#include "lattice_trajectory1d.hpp"
#include "curves/quintic_polynomial.hpp"
#define private public
#include "frenet_lattice_planner.hpp"
#undef private
namespace planning {

TEST(LatticeTrajectroyTest, generate_lattice_trajectory1d_test) {
  std::array<double, 3> init_d{-1.43473, 0.4, 0.0};
  std::array<double, 3> end_d{0.5, 0, 0};
  double param = 20.0;
  std::shared_ptr<common::Polynomial> curve = std::make_shared<common::QuinticPolynomial>(init_d, end_d, param);
  auto lattice_trajectory = std::make_shared<LatticeTrajectory1d>(curve);
  for (double s = 0; s < param + 5; s += 1) {
    std::cout << "s: " << s << ", l: " << lattice_trajectory->Evaluate(0, s)
              << ", l_prime: "
              << lattice_trajectory->Evaluate(1, s) << ", l_prime_prime: "
              << lattice_trajectory->Evaluate(2, s)
              << std::endl;

  }
}

TEST(LatticeTrajectoryTest, lattice_trajectory1d_evaluate) {
  std::array<double, 3> init_s{-1.43473, 0.4, 0.0};
  std::array<double, 3> end_s{4.0, 0.3, 0};
  double T = 3.0;
  std::shared_ptr<common::Polynomial> curve = std::make_shared<common::QuinticPolynomial>(init_s, end_s, T);
  auto lattice_trajectory = std::make_shared<LatticeTrajectory1d>(curve);

  for (double t = 0; t < T + 3.0; t += 1.0) {
    std::cout << "t: " << t << ", s: " << lattice_trajectory->Evaluate(0, t)
              << ", s_dot: "
              << lattice_trajectory->Evaluate(1, t) << ", s_ddot: "
              << lattice_trajectory->Evaluate(2, t)
              << std::endl;

  }
}

TEST(LatticeTrajectoryTest, lattice_planner_trajectory_generator) {
  std::array<double, 3> init_s{-1.43473, 0.4, 0.0};
  std::array<double, 3> end_s{2.0, 0.3, 0};
  std::vector<std::pair<std::array<double, 3>, double>> end_conditions;
  end_conditions.emplace_back(end_s, 3.0);
  double T = 3.0;
  auto lattice_planner = FrenetLatticePlanner();
  std::vector<std::shared_ptr<common::Polynomial>> lattice_trajectorys;
  FrenetLatticePlanner::GeneratePolynomialTrajectories(init_s, end_conditions, 5, &lattice_trajectorys);
  for (const auto &traj : lattice_trajectorys) {
    for (double t = 0; t < 5.0; t += 1.0) {
      std::cout << "t: " << t << ", s: " << traj->Evaluate(0, t)
                << ", s_dot: "
                << traj->Evaluate(1, t) << ", s_ddot: "
                << traj->Evaluate(2, t)
                << std::endl;
    }
  }
}

}
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



