#ifndef KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_HPP_
#define KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_HPP_

#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_A.hpp"
#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_C.hpp"
#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_measurement_equation.hpp"
#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_measurement_equation_jacobian.hpp"
#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_parameter.hpp"
#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_state_equation.hpp"
#include "kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_state_equation_jacobian.hpp"

#include "python_control.hpp"

namespace kinematic_bicycle_model_op_en_nonlinear_mpc_ekf {

using namespace PythonNumpy;
using namespace PythonControl;

constexpr std::size_t NUMBER_OF_DELAY = 0;

using A_Type = kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_A::type;

using C_Type = kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_C::type;

constexpr std::size_t STATE_SIZE = A_Type::ROWS;
constexpr std::size_t INPUT_SIZE = 2;
constexpr std::size_t OUTPUT_SIZE = C_Type::ROWS;

using X_Type = StateSpaceState_Type<float, STATE_SIZE>;
using U_Type = StateSpaceInput_Type<float, INPUT_SIZE>;
using Y_Type = StateSpaceOutput_Type<float, OUTPUT_SIZE>;

using Q_Type = KalmanFilter_Q_Type<float, STATE_SIZE>;

using R_Type = KalmanFilter_R_Type<float, OUTPUT_SIZE>;

using Parameter_Type =
    kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_parameter::Parameter_Type;

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

  StateEquation_Object<X_Type, U_Type, Parameter_Type> state_equation_object =
      [](const X_Type &X, const U_Type &U, const Parameter_Type &Parameters) {
        return kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_state_equation::
            function(X, U, Parameters);
      };

  StateEquationJacobian_Object<A_Type, X_Type, U_Type, Parameter_Type>
      state_equation_jacobian_object = [](const X_Type &X, const U_Type &U,
                                          const Parameter_Type &Parameters) {
        return kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_state_equation_jacobian::
            function(X, U, Parameters);
      };

  MeasurementEquation_Object<Y_Type, X_Type, Parameter_Type>
      measurement_equation_object = [](const X_Type &X,
                                       const Parameter_Type &Parameters) {
        return kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_measurement_equation::
            function(X, Parameters);
      };

  MeasurementEquationJacobian_Object<C_Type, X_Type, Parameter_Type>
      measurement_equation_jacobian_object = [](const X_Type &X,
                                                const Parameter_Type
                                                    &Parameters) {
        return kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_measurement_equation_jacobian::
            function(X, Parameters);
      };

  return ExtendedKalmanFilter_Type<A_Type, C_Type, U_Type, Q_Type, R_Type,
                                   Parameter_Type, NUMBER_OF_DELAY>(
      Q, R, state_equation_object, state_equation_jacobian_object,
      measurement_equation_object, measurement_equation_jacobian_object,
      parameters);
}

} // namespace kinematic_bicycle_model_op_en_nonlinear_mpc_ekf

#endif // KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_HPP_
