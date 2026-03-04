#ifndef __KINEMATIC_BICYCLE_MODEL_SQP_MEASUREMENT_FUNCTION_HPP__
#define __KINEMATIC_BICYCLE_MODEL_SQP_MEASUREMENT_FUNCTION_HPP__

#include "python_math.hpp"

namespace kinematic_bicycle_model_sqp_measurement_function {

using namespace PythonMath;

template <typename X_Type, typename U_Type, typename Parameter_Type,
          typename Y_Type>
class Function {
public:
  static inline auto sympy_function(const float py, const float q3,
                                    const float px, const float q0) -> Y_Type {

    Y_Type result;

    result.template set<0, 0>(static_cast<float>(px));
    result.template set<1, 0>(static_cast<float>(py));
    result.template set<2, 0>(static_cast<float>(q0));
    result.template set<3, 0>(static_cast<float>(q3));

    return result;
  }

  static inline auto function(const X_Type X, const U_Type U,
                              const Parameter_Type Parameters) -> Y_Type {

    float px = X.template get<0, 0>();

    float py = X.template get<1, 0>();

    float q0 = X.template get<2, 0>();

    float q3 = X.template get<3, 0>();

    return sympy_function(py, q3, px, q0);
  }
};

} // namespace kinematic_bicycle_model_sqp_measurement_function

#endif // __KINEMATIC_BICYCLE_MODEL_SQP_MEASUREMENT_FUNCTION_HPP__
