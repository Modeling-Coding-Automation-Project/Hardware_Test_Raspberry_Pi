#ifndef __KINEMATIC_BICYCLE_MODEL_SQP_STATE_JACOBIAN_X_HPP__
#define __KINEMATIC_BICYCLE_MODEL_SQP_STATE_JACOBIAN_X_HPP__

#include "python_math.hpp"
#include "python_numpy.hpp"

namespace kinematic_bicycle_model_sqp_state_jacobian_x {

using namespace PythonMath;
using namespace PythonNumpy;

using State_Jacobian_x_Type_SparseAvailable =
    SparseAvailable<ColumnAvailable<true, false, true, false>,
                    ColumnAvailable<false, true, true, true>,
                    ColumnAvailable<false, false, true, true>,
                    ColumnAvailable<false, false, true, true>>;

using State_Jacobian_x_Type =
    SparseMatrix_Type<float, State_Jacobian_x_Type_SparseAvailable>;

template <typename X_Type, typename U_Type, typename Parameter_Type>
class Function {
public:
  static inline auto sympy_function(const float q0, const float v,
                                    const float delta, const float delta_time,
                                    const float wheel_base, const float q3)
      -> State_Jacobian_x_Type {

    State_Jacobian_x_Type result;

    float x0 = delta_time * v;

    float x1 = q0 * x0;

    float x2 = x0 * tan(delta) / (2 * wheel_base);

    float x3 = cos(x2);

    float x4 = sin(x2);

    result.template set<0, 0>(static_cast<float>(1));
    result.template set<0, 1>(static_cast<float>(0));
    result.template set<0, 2>(static_cast<float>(4 * x1));
    result.template set<0, 3>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(0));
    result.template set<1, 1>(static_cast<float>(1));
    result.template set<1, 2>(static_cast<float>(2 * q3 * x0));
    result.template set<1, 3>(static_cast<float>(2 * x1));
    result.template set<2, 0>(static_cast<float>(0));
    result.template set<2, 1>(static_cast<float>(0));
    result.template set<2, 2>(static_cast<float>(x3));
    result.template set<2, 3>(static_cast<float>(-x4));
    result.template set<3, 0>(static_cast<float>(0));
    result.template set<3, 1>(static_cast<float>(0));
    result.template set<3, 2>(static_cast<float>(x4));
    result.template set<3, 3>(static_cast<float>(x3));

    return result;
  }

  static inline auto function(const X_Type X, const U_Type U,
                              const Parameter_Type Parameters)
      -> State_Jacobian_x_Type {

    float q0 = X.template get<2, 0>();

    float q3 = X.template get<3, 0>();

    float v = U.template get<0, 0>();

    float delta = U.template get<1, 0>();

    float delta_time = Parameters.delta_time;

    float wheel_base = Parameters.wheel_base;

    return sympy_function(q0, v, delta, delta_time, wheel_base, q3);
  }
};

} // namespace kinematic_bicycle_model_sqp_state_jacobian_x

#endif // __KINEMATIC_BICYCLE_MODEL_SQP_STATE_JACOBIAN_X_HPP__
