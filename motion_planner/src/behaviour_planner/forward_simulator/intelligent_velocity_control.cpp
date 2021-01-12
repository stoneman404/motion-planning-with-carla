#include "intelligent_velocity_control.hpp"
namespace planning {
bool IntelligentVelocityControl::CalculateDesiredVelocity(
    const IDMParams &params,
    const double s,
    const double s_front,
    const double v,
    const double v_front,
    const double dt,
    double *velocity_at_dt) {
  std::array<double, 4> state  {s, std::max(0.0, v), s_front, v_front};
  IntelligentDriverModel model(params, state);
  auto updated_state = model.Step(dt);
  *velocity_at_dt = std::max(0.0, updated_state[1]);
  return true;
}

IntelligentDriverModel::IntelligentDriverModel(const IDMParams &param, std::array<double, 4> &state)
    : param_(param), state_(state) {}

std::array<double, 4> IntelligentDriverModel::Step(double dt) {
  IntelligentDriverModel::State state = state_;
  boost::numeric::odeint::integrate(boost::ref(*this), state, 0.0, dt, dt);
  return state;
}

void IntelligentDriverModel::operator()(const State &x, State &dxdt, const double dt) {
  double acc;
  IntelligentDriverModel::GetAccDesiredAcceleration(param_, x, &acc);
  acc = std::max(acc, -std::min(param_.max_decel, x[1] / dt));
  dxdt[0] = x[1];
  dxdt[1] = acc;
  dxdt[2] = x[2];
  dxdt[3] = 0.0;
}
bool IntelligentDriverModel::GetAccDesiredAcceleration(const IDMParams &param,
                                                       const IntelligentDriverModel::State &cur_state,
                                                       double *acc) {

  double acc_iidm;
  IntelligentDriverModel::GetIIdmDesiredAcceleration(param, cur_state, &acc_iidm);

  // ~ Here we simply use a constant dec, ego comfortable dec, as the acc of
  // ~ leading vehicle.
  double ds = std::max(0.0, cur_state[2] - cur_state[0]);
  double acc_cah =
      (cur_state[1] * cur_state[1] * -param.comfortable_decel) /
          (cur_state[3] * cur_state[3] -
              2 * ds * -param.comfortable_decel);

  double coolness = 0.99;

  if (acc_iidm >= acc_cah) {
    *acc = acc_iidm;
  } else {
    *acc =
        (1 - coolness) * acc_iidm +
            coolness * (acc_cah - param.comfortable_decel *
                tanh((acc_iidm - acc_cah) /
                    -param.comfortable_decel));
  }
  return true;
}

bool IntelligentDriverModel::GetIIdmDesiredAcceleration(const IDMParams &param,
                                                        const IntelligentDriverModel::State &cur_state,
                                                        double *acc) {

  // The Improved IntelligentDriverModel (IIDM) tries to address two
  // deficiencies of the original IDM model:
  // 1) If the actual speed exceeds the desired speed (e.g., after entering a
  // zone with a reduced speed limit), the deceleration is unrealistically
  // large, particularly for large values of the acceleration exponent δ.
  // 2) Near the desired speed v0, the steady-state gap becomes much
  // greater than s∗(v, 0) = s0 + vT so that the model parameter T loses its
  // meaning as the desired time gap. This means that a platoon of identical
  // drivers and vehicles disperses much more than observed. Moreover, not all
  // cars will reach the desired speed
  double a_free =
      cur_state[1] <= param.desired_velocity
      ? param.max_acc *
          (1 - pow(cur_state[1] / param.desired_velocity, param.acc_exponent))
      : -param.comfortable_decel *
          (1 - pow(param.desired_velocity / cur_state[1],
                   param.max_acc * param.acc_exponent /
                       param.comfortable_decel));
  double s_alpha =
      std::max(0.0, cur_state[2] - cur_state[0] - param.leading_vehicle_length);
  double z =
      (param.s0 +
          std::max(0.0,
                   cur_state[1] * param.safe_time_headway +
                       cur_state[1] * (cur_state[1] - cur_state[3]) /
                           (2.0 * sqrt(param.max_acc *
                               param.comfortable_decel)))) /
          s_alpha;
  double a_out =
      cur_state[1] <= param.desired_velocity
      ? (z >= 1.0
         ? param.max_acc * (1 - pow(z, 2))
         : a_free * (1 - pow(z, 2.0 * param.max_acc / a_free)))
      : (z >= 1.0 ? a_free + param.max_acc * (1 - pow(z, 2))
                  : a_free);
  a_out = std::max(std::min(param.max_acc, a_out),
                   -param.max_decel);
  *acc = a_out;
  return true;
}

}

