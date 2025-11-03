#ifndef __KINEMATIC_BICYCLE_MODEL_COST_MATRICES_HPP__
#define __KINEMATIC_BICYCLE_MODEL_COST_MATRICES_HPP__

#include "kinematic_bicycle_model_parameter.hpp"
#include "kinematic_bicycle_model_sqp_hessian_f_uu.hpp"
#include "kinematic_bicycle_model_sqp_hessian_f_ux.hpp"
#include "kinematic_bicycle_model_sqp_hessian_f_xu.hpp"
#include "kinematic_bicycle_model_sqp_hessian_f_xx.hpp"
#include "kinematic_bicycle_model_sqp_hessian_h_xx.hpp"
#include "kinematic_bicycle_model_sqp_measurement_function.hpp"
#include "kinematic_bicycle_model_sqp_measurement_jacobian_x.hpp"
#include "kinematic_bicycle_model_sqp_state_function.hpp"
#include "kinematic_bicycle_model_sqp_state_jacobian_u.hpp"
#include "kinematic_bicycle_model_sqp_state_jacobian_x.hpp"

#include "kinematic_bicycle_model_cost_matrices_U_max.hpp"
#include "kinematic_bicycle_model_cost_matrices_U_min.hpp"
#include "kinematic_bicycle_model_cost_matrices_Y_max.hpp"
#include "kinematic_bicycle_model_cost_matrices_Y_min.hpp"

#include "python_optimization.hpp"

namespace kinematic_bicycle_model_cost_matrices {

using namespace PythonNumpy;
using namespace PythonControl;
using namespace PythonOptimization;

constexpr std::size_t NP = 10;

constexpr std::size_t INPUT_SIZE = 2;
constexpr std::size_t STATE_SIZE = 4;
constexpr std::size_t OUTPUT_SIZE = 4;

using X_Type = StateSpaceState_Type<float, STATE_SIZE>;
using U_Type = StateSpaceInput_Type<float, INPUT_SIZE>;
using Y_Type = StateSpaceOutput_Type<float, OUTPUT_SIZE>;

using Parameter_Type = kinematic_bicycle_model_parameter::Parameter;

using State_Jacobian_X_Matrix_Type =
    kinematic_bicycle_model_sqp_state_jacobian_x::State_Jacobian_x_Type;
using State_Jacobian_U_Matrix_Type =
    kinematic_bicycle_model_sqp_state_jacobian_u::State_Jacobian_u_Type;
using Measurement_Jacobian_X_Matrix_Type =
    kinematic_bicycle_model_sqp_measurement_jacobian_x::
        Measurement_Jacobian_x_Type;

using State_Hessian_XX_Matrix_Type =
    kinematic_bicycle_model_sqp_hessian_f_xx::State_Hessian_xx_Type;
using State_Hessian_XU_Matrix_Type =
    kinematic_bicycle_model_sqp_hessian_f_xu::State_Hessian_xu_Type;
using State_Hessian_UX_Matrix_Type =
    kinematic_bicycle_model_sqp_hessian_f_ux::State_Hessian_ux_Type;
using State_Hessian_UU_Matrix_Type =
    kinematic_bicycle_model_sqp_hessian_f_uu::State_Hessian_uu_Type;
using Measurement_Hessian_XX_Matrix_Type =
    kinematic_bicycle_model_sqp_hessian_h_xx::Measurement_Hessian_xx_Type;

using Qx_Type = DiagMatrix_Type<float, STATE_SIZE>;
using R_Type = DiagMatrix_Type<float, INPUT_SIZE>;
using Qy_Type = DiagMatrix_Type<float, OUTPUT_SIZE>;

using U_Min_Type = kinematic_bicycle_model_cost_matrices_U_min::type;
using U_Max_Type = kinematic_bicycle_model_cost_matrices_U_max::type;
using Y_Min_Type = kinematic_bicycle_model_cost_matrices_Y_min::type;
using Y_Max_Type = kinematic_bicycle_model_cost_matrices_Y_max::type;

using Reference_Trajectory_Type =
    DenseMatrix_Type<float, OUTPUT_SIZE, (NP + 1)>;

using type = SQP_CostMatrices_NMPC_Type<
    float, NP, Parameter_Type, U_Min_Type, U_Max_Type, Y_Min_Type, Y_Max_Type,
    State_Jacobian_X_Matrix_Type, State_Jacobian_U_Matrix_Type,
    Measurement_Jacobian_X_Matrix_Type, State_Hessian_XX_Matrix_Type,
    State_Hessian_XU_Matrix_Type, State_Hessian_UX_Matrix_Type,
    State_Hessian_UU_Matrix_Type, Measurement_Hessian_XX_Matrix_Type>;

inline auto make() -> type {

  auto U_min = kinematic_bicycle_model_cost_matrices_U_min::make();

  U_min.template set<0, 0>(static_cast<float>(-1.0));
  U_min.template set<1, 0>(static_cast<float>(-1.5));

  auto U_max = kinematic_bicycle_model_cost_matrices_U_max::make();

  U_max.template set<0, 0>(static_cast<float>(1.0));
  U_max.template set<1, 0>(static_cast<float>(1.5));

  auto Y_min = kinematic_bicycle_model_cost_matrices_Y_min::make();

  auto Y_max = kinematic_bicycle_model_cost_matrices_Y_max::make();

  Qx_Type Qx = make_DiagMatrix<STATE_SIZE>(
      static_cast<float>(0.0), static_cast<float>(0.0), static_cast<float>(0.0),
      static_cast<float>(0.0));

  R_Type R = make_DiagMatrix<INPUT_SIZE>(static_cast<float>(0.05),
                                         static_cast<float>(0.05));

  Qy_Type Qy = make_DiagMatrix<OUTPUT_SIZE>(
      static_cast<float>(1.0), static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0));

  Reference_Trajectory_Type reference_trajectory;

  type cost_matrices = make_SQP_CostMatrices_NMPC<
      float, NP, Parameter_Type, U_Min_Type, U_Max_Type, Y_Min_Type, Y_Max_Type,
      State_Jacobian_X_Matrix_Type, State_Jacobian_U_Matrix_Type,
      Measurement_Jacobian_X_Matrix_Type, State_Hessian_XX_Matrix_Type,
      State_Hessian_XU_Matrix_Type, State_Hessian_UX_Matrix_Type,
      State_Hessian_UU_Matrix_Type, Measurement_Hessian_XX_Matrix_Type>(
      Qx, R, Qy, U_min, U_max, Y_min, Y_max);

  PythonOptimization::StateFunction_Object<X_Type, U_Type, Parameter_Type>
      state_function = kinematic_bicycle_model_sqp_state_function::Function<
          X_Type, U_Type, Parameter_Type>::function;

  PythonOptimization::MeasurementFunction_Object<Y_Type, X_Type, U_Type,
                                                 Parameter_Type>
      measurement_function =
          kinematic_bicycle_model_sqp_measurement_function::Function<
              X_Type, U_Type, Parameter_Type, Y_Type>::function;

  PythonOptimization::StateFunctionJacobian_X_Object<
      State_Jacobian_X_Matrix_Type, X_Type, U_Type, Parameter_Type>
      state_jacobian_x_function =
          kinematic_bicycle_model_sqp_state_jacobian_x::Function<
              X_Type, U_Type, Parameter_Type>::function;

  PythonOptimization::StateFunctionJacobian_U_Object<
      State_Jacobian_U_Matrix_Type, X_Type, U_Type, Parameter_Type>
      state_jacobian_u_function =
          kinematic_bicycle_model_sqp_state_jacobian_u::Function<
              X_Type, U_Type, Parameter_Type>::function;

  PythonOptimization::MeasurementFunctionJacobian_X_Object<
      Measurement_Jacobian_X_Matrix_Type, X_Type, U_Type, Parameter_Type>
      measurement_jacobian_x_function =
          kinematic_bicycle_model_sqp_measurement_jacobian_x::Function<
              X_Type, U_Type, Parameter_Type>::function;

  PythonOptimization::StateFunctionHessian_XX_Object<
      State_Hessian_XX_Matrix_Type, X_Type, U_Type, Parameter_Type>
      state_hessian_xx_function =
          kinematic_bicycle_model_sqp_hessian_f_xx::Function<
              X_Type, U_Type, Parameter_Type>::function;

  PythonOptimization::StateFunctionHessian_XU_Object<
      State_Hessian_XU_Matrix_Type, X_Type, U_Type, Parameter_Type>
      state_hessian_xu_function =
          kinematic_bicycle_model_sqp_hessian_f_xu::Function<
              X_Type, U_Type, Parameter_Type>::function;

  PythonOptimization::StateFunctionHessian_UX_Object<
      State_Hessian_UX_Matrix_Type, X_Type, U_Type, Parameter_Type>
      state_hessian_ux_function =
          kinematic_bicycle_model_sqp_hessian_f_ux::Function<
              X_Type, U_Type, Parameter_Type>::function;

  PythonOptimization::StateFunctionHessian_UU_Object<
      State_Hessian_UU_Matrix_Type, X_Type, U_Type, Parameter_Type>
      state_hessian_uu_function =
          kinematic_bicycle_model_sqp_hessian_f_uu::Function<
              X_Type, U_Type, Parameter_Type>::function;

  PythonOptimization::MeasurementFunctionHessian_XX_Object<
      Measurement_Hessian_XX_Matrix_Type, X_Type, U_Type, Parameter_Type>
      measurement_hessian_xx_function =
          kinematic_bicycle_model_sqp_hessian_h_xx::Function<
              X_Type, U_Type, Parameter_Type>::function;

  cost_matrices.set_function_objects(
      state_function, measurement_function, state_jacobian_x_function,
      state_jacobian_u_function, measurement_jacobian_x_function,
      state_hessian_xx_function, state_hessian_xu_function,
      state_hessian_ux_function, state_hessian_uu_function,
      measurement_hessian_xx_function);

  return cost_matrices;
}

} // namespace kinematic_bicycle_model_cost_matrices

#endif // __KINEMATIC_BICYCLE_MODEL_COST_MATRICES_HPP__
