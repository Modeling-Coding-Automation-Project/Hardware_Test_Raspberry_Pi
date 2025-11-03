#ifndef __KINEMATIC_BICYCLE_MODEL_COST_MATRICES_U_MIN_HPP__
#define __KINEMATIC_BICYCLE_MODEL_COST_MATRICES_U_MIN_HPP__

#include "python_numpy.hpp"

namespace kinematic_bicycle_model_cost_matrices_U_min {

using namespace PythonNumpy;

using type = DenseMatrix_Type<float, 2, 1>;

inline auto make(void) -> type {

  return make_DenseMatrix<2, 1>(static_cast<float>(1.0),
                                static_cast<float>(1.0));
}

} // namespace kinematic_bicycle_model_cost_matrices_U_min

#endif // __KINEMATIC_BICYCLE_MODEL_COST_MATRICES_U_MIN_HPP__
