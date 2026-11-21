#ifndef KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_C_HPP_
#define KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_C_HPP_

#include "python_numpy.hpp"

namespace kinematic_bicycle_model_nonlinear_mpc_ekf_C {

using namespace PythonNumpy;

using type = DiagMatrix_Type<float, 4>;

inline auto make(void) -> type {

  auto result = make_DiagMatrixZeros<float, 4>();

  result.unsafe_access(0) = static_cast<float>(1.0);
  result.unsafe_access(1) = static_cast<float>(1.0);
  result.unsafe_access(2) = static_cast<float>(1.0);
  result.unsafe_access(3) = static_cast<float>(1.0);

  return result;
}

} // namespace kinematic_bicycle_model_nonlinear_mpc_ekf_C

#endif // KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_C_HPP_
