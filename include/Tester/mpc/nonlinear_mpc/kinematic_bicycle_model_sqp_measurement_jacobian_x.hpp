#ifndef __KINEMATIC_BICYCLE_MODEL_SQP_MEASUREMENT_JACOBIAN_X_HPP__
#define __KINEMATIC_BICYCLE_MODEL_SQP_MEASUREMENT_JACOBIAN_X_HPP__

#include "python_math.hpp"
#include "python_numpy.hpp"

namespace kinematic_bicycle_model_sqp_measurement_jacobian_x {

using namespace PythonMath;
using namespace PythonNumpy;

using Measurement_Jacobian_x_Type_SparseAvailable =
    SparseAvailable<ColumnAvailable<true, false, false, false>,
                    ColumnAvailable<false, true, false, false>,
                    ColumnAvailable<false, false, true, false>,
                    ColumnAvailable<false, false, false, true>>;

using Measurement_Jacobian_x_Type =
    SparseMatrix_Type<float, Measurement_Jacobian_x_Type_SparseAvailable>;

template <typename X_Type, typename U_Type, typename Parameter_Type>
class Function {
public:
  static inline auto sympy_function() -> Measurement_Jacobian_x_Type {

    Measurement_Jacobian_x_Type result;

    result.template set<0, 0>(static_cast<float>(1));
    result.template set<0, 1>(static_cast<float>(0));
    result.template set<0, 2>(static_cast<float>(0));
    result.template set<0, 3>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(0));
    result.template set<1, 1>(static_cast<float>(1));
    result.template set<1, 2>(static_cast<float>(0));
    result.template set<1, 3>(static_cast<float>(0));
    result.template set<2, 0>(static_cast<float>(0));
    result.template set<2, 1>(static_cast<float>(0));
    result.template set<2, 2>(static_cast<float>(1));
    result.template set<2, 3>(static_cast<float>(0));
    result.template set<3, 0>(static_cast<float>(0));
    result.template set<3, 1>(static_cast<float>(0));
    result.template set<3, 2>(static_cast<float>(0));
    result.template set<3, 3>(static_cast<float>(1));

    return result;
  }

  static inline auto function(const X_Type X, const U_Type U,
                              const Parameter_Type Parameters)
      -> Measurement_Jacobian_x_Type {

    return sympy_function();
  }
};

} // namespace kinematic_bicycle_model_sqp_measurement_jacobian_x

#endif // __KINEMATIC_BICYCLE_MODEL_SQP_MEASUREMENT_JACOBIAN_X_HPP__
