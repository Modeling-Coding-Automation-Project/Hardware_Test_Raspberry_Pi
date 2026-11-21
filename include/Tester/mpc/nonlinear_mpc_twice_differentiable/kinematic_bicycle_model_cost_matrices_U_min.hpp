#ifndef KINEMATIC_BICYCLE_MODEL_COST_MATRICES_U_MIN_HPP_
#define KINEMATIC_BICYCLE_MODEL_COST_MATRICES_U_MIN_HPP_

#include "python_numpy.hpp"

namespace kinematic_bicycle_model_cost_matrices_U_min {

using namespace PythonNumpy;

using type = DenseMatrix_Type<float, 2, 1>;

inline auto make(void) -> type {

  auto result = make_DenseMatrixZeros<float, 2, 1>();

  result.unsafe_access(0, 0) = static_cast<float>(1.0);
  result.unsafe_access(1, 0) = static_cast<float>(1.0);

  return result;
}

} // namespace kinematic_bicycle_model_cost_matrices_U_min

#endif // KINEMATIC_BICYCLE_MODEL_COST_MATRICES_U_MIN_HPP_
