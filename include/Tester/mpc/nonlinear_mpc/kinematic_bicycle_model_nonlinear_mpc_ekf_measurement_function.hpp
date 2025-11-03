#ifndef __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_MEASUREMENT_FUNCTION_HPP__
#define __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_MEASUREMENT_FUNCTION_HPP__

#include "kinematic_bicycle_model_ekf_A.hpp"
#include "kinematic_bicycle_model_ekf_C.hpp"
#include "kinematic_bicycle_model_nonlinear_mpc_ekf_parameter.hpp"

#include "python_control.hpp"

namespace kinematic_bicycle_model_nonlinear_mpc_ekf_measurement_function {

using namespace PythonControl;

using Parameter_Type =
    kinematic_bicycle_model_nonlinear_mpc_ekf_parameter::Parameter_Type;

using namespace PythonMath;

using A_Type = kinematic_bicycle_model_ekf_A::type;
using C_Type = kinematic_bicycle_model_ekf_C::type;
using X_Type = StateSpaceState_Type<float, A_Type::COLS>;
using Y_Type = StateSpaceOutput_Type<float, C_Type::COLS>;

inline auto sympy_function(const float py, const float q3, const float px,
                           const float q0) -> Y_Type {

  Y_Type result;

  result.template set<0, 0>(static_cast<float>(px));
  result.template set<1, 0>(static_cast<float>(py));
  result.template set<2, 0>(static_cast<float>(q0));
  result.template set<3, 0>(static_cast<float>(q3));

  return result;
}

inline auto function(const X_Type X, const Parameter_Type Parameters)
    -> Y_Type {

  float px = X.template get<0, 0>();

  float py = X.template get<1, 0>();

  float q0 = X.template get<2, 0>();

  float q3 = X.template get<3, 0>();

  return sympy_function(py, q3, px, q0);
}

} // namespace kinematic_bicycle_model_nonlinear_mpc_ekf_measurement_function

#endif // __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_MEASUREMENT_FUNCTION_HPP__
