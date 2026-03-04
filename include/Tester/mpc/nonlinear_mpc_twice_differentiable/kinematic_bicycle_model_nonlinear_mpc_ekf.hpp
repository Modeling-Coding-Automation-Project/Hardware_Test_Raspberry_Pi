#ifndef __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_HPP__
#define __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_HPP__

#include "kinematic_bicycle_model_ekf_A.hpp"
#include "kinematic_bicycle_model_ekf_C.hpp"
#include "kinematic_bicycle_model_nonlinear_mpc_ekf_measurement_function.hpp"
#include "kinematic_bicycle_model_nonlinear_mpc_ekf_measurement_function_jacobian.hpp"
#include "kinematic_bicycle_model_nonlinear_mpc_ekf_parameter.hpp"
#include "kinematic_bicycle_model_nonlinear_mpc_ekf_state_function.hpp"
#include "kinematic_bicycle_model_nonlinear_mpc_ekf_state_function_jacobian.hpp"

#include "python_control.hpp"

namespace kinematic_bicycle_model_nonlinear_mpc_ekf {

using namespace PythonNumpy;
using namespace PythonControl;

constexpr std::size_t NUMBER_OF_DELAY = 0;

using A_Type = kinematic_bicycle_model_ekf_A::type;

using C_Type = kinematic_bicycle_model_ekf_C::type;

constexpr std::size_t STATE_SIZE = A_Type::COLS;
constexpr std::size_t INPUT_SIZE = 2;
constexpr std::size_t OUTPUT_SIZE = C_Type::COLS;

using X_Type = StateSpaceState_Type<float, STATE_SIZE>;
using U_Type = StateSpaceInput_Type<float, INPUT_SIZE>;
using Y_Type = StateSpaceOutput_Type<float, OUTPUT_SIZE>;

using Q_Type = KalmanFilter_Q_Type<float, STATE_SIZE>;

using R_Type = KalmanFilter_R_Type<float, OUTPUT_SIZE>;

using Parameter_Type =
    kinematic_bicycle_model_nonlinear_mpc_ekf_parameter::Parameter_Type;

using type = ExtendedKalmanFilter_Type<A_Type, C_Type, U_Type, Q_Type, R_Type,
                                       Parameter_Type, NUMBER_OF_DELAY>;

inline auto make() -> type {

  auto Q = make_KalmanFilter_Q<STATE_SIZE>(
      static_cast<float>(1.0), static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0));

  auto R = make_KalmanFilter_R<OUTPUT_SIZE>(
      static_cast<float>(1.0), static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0));

  Parameter_Type parameters;

  StateFunction_Object<X_Type, U_Type, Parameter_Type> state_function_object =
      [](const X_Type &X, const U_Type &U, const Parameter_Type &Parameters) {
        return kinematic_bicycle_model_nonlinear_mpc_ekf_state_function::
            function(X, U, Parameters);
      };

  StateFunctionJacobian_Object<A_Type, X_Type, U_Type, Parameter_Type>
      state_function_jacobian_object = [](const X_Type &X, const U_Type &U,
                                          const Parameter_Type &Parameters) {
        return kinematic_bicycle_model_nonlinear_mpc_ekf_state_function_jacobian::
            function(X, U, Parameters);
      };

  MeasurementFunction_Object<Y_Type, X_Type, Parameter_Type>
      measurement_function_object = [](const X_Type &X,
                                       const Parameter_Type &Parameters) {
        return kinematic_bicycle_model_nonlinear_mpc_ekf_measurement_function::
            function(X, Parameters);
      };

  MeasurementFunctionJacobian_Object<C_Type, X_Type, Parameter_Type>
      measurement_function_jacobian_object = [](const X_Type &X,
                                                const Parameter_Type
                                                    &Parameters) {
        return kinematic_bicycle_model_nonlinear_mpc_ekf_measurement_function_jacobian::
            function(X, Parameters);
      };

  return ExtendedKalmanFilter_Type<A_Type, C_Type, U_Type, Q_Type, R_Type,
                                   Parameter_Type, NUMBER_OF_DELAY>(
      Q, R, state_function_object, state_function_jacobian_object,
      measurement_function_object, measurement_function_jacobian_object,
      parameters);
}

} // namespace kinematic_bicycle_model_nonlinear_mpc_ekf

#endif // __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_HPP__
