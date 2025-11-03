#ifndef __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_HPP__
#define __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_HPP__

#include "kinematic_bicycle_model_cost_matrices.hpp"
#include "kinematic_bicycle_model_nonlinear_mpc_ekf.hpp"

#include "python_mpc.hpp"

namespace kinematic_bicycle_model_nonlinear_mpc {

using namespace PythonNumpy;
using namespace PythonControl;
using namespace PythonMPC;

constexpr std::size_t NP = 10;

constexpr std::size_t INPUT_SIZE =
    kinematic_bicycle_model_nonlinear_mpc_ekf::INPUT_SIZE;
constexpr std::size_t STATE_SIZE =
    kinematic_bicycle_model_nonlinear_mpc_ekf::STATE_SIZE;
constexpr std::size_t OUTPUT_SIZE =
    kinematic_bicycle_model_nonlinear_mpc_ekf::OUTPUT_SIZE;

constexpr std::size_t NUMBER_OF_DELAY =
    kinematic_bicycle_model_nonlinear_mpc_ekf::NUMBER_OF_DELAY;

using X_Type = StateSpaceState_Type<float, STATE_SIZE>;

using U_Type = StateSpaceInput_Type<float, INPUT_SIZE>;

using Y_Type = StateSpaceOutput_Type<float, OUTPUT_SIZE>;

using EKF_Type = kinematic_bicycle_model_nonlinear_mpc_ekf::type;

using Parameter_Type =
    kinematic_bicycle_model_nonlinear_mpc_ekf_parameter::Parameter_Type;

using Cost_Matrices_Type = kinematic_bicycle_model_cost_matrices::type;

using Reference_Type = DenseMatrix_Type<float, OUTPUT_SIZE, 1>;

using ReferenceTrajectory_Type = DenseMatrix_Type<float, OUTPUT_SIZE, NP>;

using type =
    NonlinearMPC_TwiceDifferentiable_Type<EKF_Type, Cost_Matrices_Type>;

inline auto make(void) -> type {

  auto kalman_filter = kinematic_bicycle_model_nonlinear_mpc_ekf::make();

  auto cost_matrices = kinematic_bicycle_model_cost_matrices::make();

  float delta_time = static_cast<float>(0.1);

  X_Type X_initial;
  X_initial.template set<0, 0>(static_cast<float>(0.0));
  X_initial.template set<1, 0>(static_cast<float>(0.0));
  X_initial.template set<2, 0>(static_cast<float>(3.2679489653813835e-07));
  X_initial.template set<3, 0>(static_cast<float>(-0.9999999999999466));

  auto nonlinear_mpc = make_NonlinearMPC_TwiceDifferentiable(
      kalman_filter, cost_matrices, delta_time, X_initial);

  return nonlinear_mpc;
}

} // namespace kinematic_bicycle_model_nonlinear_mpc

#endif // __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_HPP__
