#ifndef __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_STATE_FUNCTION_JACOBIAN_HPP__
#define __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_STATE_FUNCTION_JACOBIAN_HPP__

#include "kinematic_bicycle_model_ekf_A.hpp"
#include "kinematic_bicycle_model_ekf_C.hpp"
#include "kinematic_bicycle_model_nonlinear_mpc_ekf_parameter.hpp"

#include "python_control.hpp"

namespace kinematic_bicycle_model_nonlinear_mpc_ekf_state_function_jacobian {

using namespace PythonControl;

using Parameter_Type =
    kinematic_bicycle_model_nonlinear_mpc_ekf_parameter::Parameter_Type;

using namespace PythonMath;

using A_Type = kinematic_bicycle_model_ekf_A::type;
using X_Type = StateSpaceState_Type<float, A_Type::COLS>;
using U_Type = StateSpaceInput_Type<float, 2>;

inline auto sympy_function(const float q0, const float v, const float delta,
                           const float delta_time, const float wheel_base,
                           const float q3) -> A_Type {

  A_Type result;

  float x0 = delta_time * v;

  float x1 = q0 * x0;

  float x2 = x0 * tan(delta) / (static_cast<float>(2) * wheel_base);

  float x3 = cos(x2);

  float x4 = sin(x2);

  result.template set<0, 0>(static_cast<float>(1));
  result.template set<0, 1>(static_cast<float>(0));
  result.template set<0, 2>(static_cast<float>(4 * x1));
  result.template set<0, 3>(static_cast<float>(0));
  result.template set<1, 0>(static_cast<float>(0));
  result.template set<1, 1>(static_cast<float>(1));
  result.template set<1, 2>(static_cast<float>(2 * q3 * x0));
  result.template set<1, 3>(static_cast<float>(2 * x1));
  result.template set<2, 0>(static_cast<float>(0));
  result.template set<2, 1>(static_cast<float>(0));
  result.template set<2, 2>(static_cast<float>(x3));
  result.template set<2, 3>(static_cast<float>(-x4));
  result.template set<3, 0>(static_cast<float>(0));
  result.template set<3, 1>(static_cast<float>(0));
  result.template set<3, 2>(static_cast<float>(x4));
  result.template set<3, 3>(static_cast<float>(x3));

  return result;
}

inline auto function(const X_Type X, const U_Type U,
                     const Parameter_Type Parameters) -> A_Type {

  float q0 = X.template get<2, 0>();

  float q3 = X.template get<3, 0>();

  float v = U.template get<0, 0>();

  float delta = U.template get<1, 0>();

  float delta_time = Parameters.delta_time;

  float wheel_base = Parameters.wheel_base;

  return sympy_function(q0, v, delta, delta_time, wheel_base, q3);
}

} // namespace kinematic_bicycle_model_nonlinear_mpc_ekf_state_function_jacobian

#endif // __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_STATE_FUNCTION_JACOBIAN_HPP__
