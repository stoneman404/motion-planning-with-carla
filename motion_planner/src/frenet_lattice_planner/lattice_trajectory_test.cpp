#include <gtest/gtest.h>
#include "lattice_trajectory1d.hpp"
#include "curves/quintic_polynomial.hpp"
#define private public
#include "frenet_lattice_planner.hpp"
#undef private

#include <boost/numeric/odeint.hpp>

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

TEST(LatticeTrajectoryTest, lon_trajectory_test) {
  std::array<double, 3> init_s{30.0458, 15.0, 0};
  std::array<double, 3> end_s{35.5136, 0.0, 0.0};
  std::shared_ptr<common::Polynomial> curve = std::make_shared<common::QuinticPolynomial>(init_s, end_s, 4.0);
  EXPECT_DOUBLE_EQ(curve->ParamLength(), 4.0);
  EXPECT_EQ(curve->Order(), 5);
  auto lattice_trajectory = std::make_shared<LatticeTrajectory1d>(curve);
  for (double t = 0.0; t < 7.0; t += 0.1) {
    std::cout << "t: " << t << ", s: " << lattice_trajectory->Evaluate(0, t)
              << ", s_dot: "
              << curve->Evaluate(1, t) << ", s_ddot: "
              << curve->Evaluate(2, t)
              << std::endl;
  }
  std::cout << " ---------------" << std::endl;
  for (double t = 0.0; t < 8.0; t += 0.1) {
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
typedef boost::array<double, 3> state_type;
const double sigma = 10.0;
const double R = 28.0;
const double b = 8.0 / 3.0;

using namespace std;
void lorenz(const state_type &x, state_type &dxdt, double t) {
  dxdt[0] = sigma * (x[1] - x[0]);
  dxdt[1] = R * x[0] - x[1] - x[0] * x[2];
  dxdt[2] = -b * x[2] + x[0] * x[1];
}

void write_lorenz(const state_type &x, const double t) {
  cout << t << '\t' << x[0] << '\t' << x[1] << '\t' << x[2] << endl;
}

TEST(BoostTest, odeint_test) {
  state_type x = { 10.0 , 1.0 , 1.0 }; // initial conditions
  boost::numeric::odeint::integrate( lorenz , x , 0.0 , 25.0 , 0.1 , write_lorenz );
  std::cout << x[0] << x[1] << std::endl;
}




}
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



