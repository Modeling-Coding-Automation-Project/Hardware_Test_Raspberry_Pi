#ifndef KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_A_HPP_
#define KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_A_HPP_

#include "python_numpy.hpp"

namespace kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_A {

using namespace PythonNumpy;

using SparseAvailable_nonlinear_mpc_ekf_A =
    SparseAvailable<ColumnAvailable<true, false, true, false>,
                    ColumnAvailable<false, true, true, true>,
                    ColumnAvailable<false, false, true, true>,
                    ColumnAvailable<false, false, true, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_nonlinear_mpc_ekf_A>;

inline auto make(void) -> type {

  auto result =
      make_SparseMatrixZeros<float, SparseAvailable_nonlinear_mpc_ekf_A>();

  result.unsafe_access(0) = static_cast<float>(1.0);
  result.unsafe_access(1) = static_cast<float>(1.0);
  result.unsafe_access(2) = static_cast<float>(1.0);
  result.unsafe_access(3) = static_cast<float>(1.0);
  result.unsafe_access(4) = static_cast<float>(1.0);
  result.unsafe_access(5) = static_cast<float>(1.0);
  result.unsafe_access(6) = static_cast<float>(1.0);
  result.unsafe_access(7) = static_cast<float>(1.0);
  result.unsafe_access(8) = static_cast<float>(1.0);

  return result;
}

} // namespace kinematic_bicycle_model_op_en_nonlinear_mpc_ekf_A

#endif // KINEMATIC_BICYCLE_MODEL_OP_EN_NONLINEAR_MPC_EKF_A_HPP_
