#ifndef KINEMATIC_BICYCLE_MODEL_COST_MATRICES_U_MIN_HPP_
#define KINEMATIC_BICYCLE_MODEL_COST_MATRICES_U_MIN_HPP_

#include "python_numpy.hpp"

namespace kinematic_bicycle_model_cost_matrices_U_min {

using namespace PythonNumpy;

using type = DenseMatrix_Type<float, 2, 1>;

inline auto make(void) -> type {

  return make_DenseMatrix<2, 1>(static_cast<float>(1.0),
                                static_cast<float>(1.0));
}

} // namespace kinematic_bicycle_model_cost_matrices_U_min

#endif // KINEMATIC_BICYCLE_MODEL_COST_MATRICES_U_MIN_HPP_
