#ifndef __KINEMATIC_BICYCLE_MODEL_SQP_STATE_JACOBIAN_U_HPP__
#define __KINEMATIC_BICYCLE_MODEL_SQP_STATE_JACOBIAN_U_HPP__

#include "python_math.hpp"
#include "python_numpy.hpp"

namespace kinematic_bicycle_model_sqp_state_jacobian_u {

using namespace PythonMath;
using namespace PythonNumpy;

using State_Jacobian_u_Type_SparseAvailable =
    SparseAvailable<ColumnAvailable<true, false>, ColumnAvailable<true, false>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>>;

using State_Jacobian_u_Type =
    SparseMatrix_Type<float, State_Jacobian_u_Type_SparseAvailable>;

template <typename X_Type, typename U_Type, typename Parameter_Type>
class Function {
public:
  static inline auto sympy_function(const float q0, const float v,
                                    const float delta, const float delta_time,
                                    const float wheel_base, const float q3)
      -> State_Jacobian_u_Type {

    State_Jacobian_u_Type result;

    float x0 = delta_time * q0;

    float x1 = 1 / wheel_base;

    float x2 = tan(delta);

    float x3 = x1 * x2 / 2;

    float x4 = delta_time * x3;

    float x5 = v * x4;

    float x6 = sin(x5);

    float x7 = x0 * x3;

    float x8 = cos(x5);

    float x9 = q3 * x4;

    float x10 = v * x1 * (x2 * x2 + 1) / 2;

    float x11 = x0 * x10;

    float x12 = delta_time * q3 * x10;

    result.template set<0, 0>(
        static_cast<float>(delta_time * (2 * (q0 * q0) - 1)));
    result.template set<0, 1>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(2 * q3 * x0));
    result.template set<1, 1>(static_cast<float>(0));
    result.template set<2, 0>(static_cast<float>(-x6 * x7 - x8 * x9));
    result.template set<2, 1>(static_cast<float>(-x11 * x6 - x12 * x8));
    result.template set<3, 0>(static_cast<float>(-x6 * x9 + x7 * x8));
    result.template set<3, 1>(static_cast<float>(x11 * x8 - x12 * x6));

    return result;
  }

  static inline auto function(const X_Type X, const U_Type U,
                              const Parameter_Type Parameters)
      -> State_Jacobian_u_Type {

    float q0 = X.template get<2, 0>();

    float q3 = X.template get<3, 0>();

    float v = U.template get<0, 0>();

    float delta = U.template get<1, 0>();

    float delta_time = Parameters.delta_time;

    float wheel_base = Parameters.wheel_base;

    return sympy_function(q0, v, delta, delta_time, wheel_base, q3);
  }
};

} // namespace kinematic_bicycle_model_sqp_state_jacobian_u

#endif // __KINEMATIC_BICYCLE_MODEL_SQP_STATE_JACOBIAN_U_HPP__
