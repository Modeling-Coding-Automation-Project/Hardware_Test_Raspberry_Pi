#ifndef __KINEMATIC_BICYCLE_MODEL_COST_MATRICES_Y_MAX_HPP__
#define __KINEMATIC_BICYCLE_MODEL_COST_MATRICES_Y_MAX_HPP__

#include "python_numpy.hpp"

namespace kinematic_bicycle_model_cost_matrices_Y_max {

using namespace PythonNumpy;

using SparseAvailable_cost_matrices_Y_max =
    SparseAvailable<ColumnAvailable<false>, ColumnAvailable<false>,
                    ColumnAvailable<false>, ColumnAvailable<false>>;

using type = SparseMatrix_Type<float, SparseAvailable_cost_matrices_Y_max>;

inline auto make(void) -> type { return make_SparseMatrixEmpty<float, 4, 1>(); }

} // namespace kinematic_bicycle_model_cost_matrices_Y_max

#endif // __KINEMATIC_BICYCLE_MODEL_COST_MATRICES_Y_MAX_HPP__
