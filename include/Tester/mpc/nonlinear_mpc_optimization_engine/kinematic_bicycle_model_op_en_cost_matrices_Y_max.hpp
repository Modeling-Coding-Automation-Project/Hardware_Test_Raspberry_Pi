#ifndef KINEMATIC_BICYCLE_MODEL_OP_EN_COST_MATRICES_Y_MAX_HPP_
#define KINEMATIC_BICYCLE_MODEL_OP_EN_COST_MATRICES_Y_MAX_HPP_

#include "python_numpy.hpp"

namespace kinematic_bicycle_model_op_en_cost_matrices_Y_max {

using namespace PythonNumpy;

using SparseAvailable_cost_matrices_Y_max =
    SparseAvailable<ColumnAvailable<false>, ColumnAvailable<false>,
                    ColumnAvailable<false>, ColumnAvailable<false>>;

using type = SparseMatrix_Type<float, SparseAvailable_cost_matrices_Y_max>;

inline auto make(void) -> type { return make_SparseMatrixEmpty<float, 4, 1>(); }

} // namespace kinematic_bicycle_model_op_en_cost_matrices_Y_max

#endif // KINEMATIC_BICYCLE_MODEL_OP_EN_COST_MATRICES_Y_MAX_HPP_
