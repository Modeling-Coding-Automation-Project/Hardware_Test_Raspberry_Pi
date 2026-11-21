#ifndef __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_STATE_EQUATION_HPP__
#define __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_STATE_EQUATION_HPP__

#include "kinematic_bicycle_model_nonlinear_mpc_ekf_A.hpp"
#include "kinematic_bicycle_model_nonlinear_mpc_ekf_parameter.hpp"

#include "python_control.hpp"

namespace kinematic_bicycle_model_nonlinear_mpc_ekf_state_equation {

using namespace PythonControl;

using Parameter_Type =
    kinematic_bicycle_model_nonlinear_mpc_ekf_parameter::Parameter_Type;

using namespace PythonNumpy;

using A_Type = kinematic_bicycle_model_nonlinear_mpc_ekf_A::type;
using X_Type = StateSpaceState_Type<float, A_Type::ROWS>;
using U_Type = StateSpaceInput_Type<float, 2>;

inline auto sympy_function(const float q3, const float wheel_base,
                           const float v, const float py, const float px,
                           const float delta_time, const float delta,
                           const float q0) -> X_Type {

  X_Type result;

  float x0 = delta_time * v;

  float x1 = x0 * tan(delta) / (2 * wheel_base);

  float x2 = cos(x1);

  float x3 = sin(x1);

  result.template set<0, 0>(static_cast<float>(px + x0 * (2 * (q0 * q0) - 1)));
  result.template set<1, 0>(static_cast<float>(py + 2 * q0 * q3 * x0));
  result.template set<2, 0>(static_cast<float>(q0 * x2 - q3 * x3));
  result.template set<3, 0>(static_cast<float>(q0 * x3 + q3 * x2));

  return result;
}

inline auto function(const X_Type X, const U_Type U,
                     const Parameter_Type Parameters) -> X_Type {

  float px = X.template get<0, 0>();

  float py = X.template get<1, 0>();

  float q0 = X.template get<2, 0>();

  float q3 = X.template get<3, 0>();

  float v = U.template get<0, 0>();

  float delta = U.template get<1, 0>();

  float wheel_base = Parameters.wheel_base;

  float delta_time = Parameters.delta_time;

  return sympy_function(q3, wheel_base, v, py, px, delta_time, delta, q0);
}

} // namespace kinematic_bicycle_model_nonlinear_mpc_ekf_state_equation

#endif // __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_STATE_EQUATION_HPP__
