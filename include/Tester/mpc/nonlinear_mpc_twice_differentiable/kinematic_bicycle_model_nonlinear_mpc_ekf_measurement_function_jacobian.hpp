#ifndef __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_MEASUREMENT_FUNCTION_JACOBIAN_HPP__
#define __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_MEASUREMENT_FUNCTION_JACOBIAN_HPP__

#include "kinematic_bicycle_model_ekf_A.hpp"
#include "kinematic_bicycle_model_ekf_C.hpp"
#include "kinematic_bicycle_model_nonlinear_mpc_ekf_parameter.hpp"
#include "python_control.hpp"

namespace kinematic_bicycle_model_nonlinear_mpc_ekf_measurement_function_jacobian {

using namespace PythonControl;

using Parameter_Type =
    kinematic_bicycle_model_nonlinear_mpc_ekf_parameter::Parameter_Type;

using namespace PythonMath;

using A_Type = kinematic_bicycle_model_ekf_A::type;
using C_Type = kinematic_bicycle_model_ekf_C::type;
using X_Type = StateSpaceState_Type<float, A_Type::COLS>;
using Y_Type = StateSpaceOutput_Type<float, C_Type::COLS>;

inline auto sympy_function() -> C_Type {

  C_Type result;

  result.template set<0, 0>(static_cast<float>(1));
  result.template set<0, 1>(static_cast<float>(0));
  result.template set<0, 2>(static_cast<float>(0));
  result.template set<0, 3>(static_cast<float>(0));
  result.template set<1, 0>(static_cast<float>(0));
  result.template set<1, 1>(static_cast<float>(1));
  result.template set<1, 2>(static_cast<float>(0));
  result.template set<1, 3>(static_cast<float>(0));
  result.template set<2, 0>(static_cast<float>(0));
  result.template set<2, 1>(static_cast<float>(0));
  result.template set<2, 2>(static_cast<float>(1));
  result.template set<2, 3>(static_cast<float>(0));
  result.template set<3, 0>(static_cast<float>(0));
  result.template set<3, 1>(static_cast<float>(0));
  result.template set<3, 2>(static_cast<float>(0));
  result.template set<3, 3>(static_cast<float>(1));

  return result;
}

inline auto function(const X_Type X, const Parameter_Type Parameters)
    -> C_Type {

  return sympy_function();
}

} // namespace
  // kinematic_bicycle_model_nonlinear_mpc_ekf_measurement_function_jacobian

#endif // __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_MEASUREMENT_FUNCTION_JACOBIAN_HPP__
