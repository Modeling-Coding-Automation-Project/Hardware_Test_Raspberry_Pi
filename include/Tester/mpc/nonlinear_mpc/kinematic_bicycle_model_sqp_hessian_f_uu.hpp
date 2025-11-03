#ifndef __KINEMATIC_BICYCLE_MODEL_SQP_HESSIAN_F_UU_HPP__
#define __KINEMATIC_BICYCLE_MODEL_SQP_HESSIAN_F_UU_HPP__

#include "python_math.hpp"
#include "python_numpy.hpp"

namespace kinematic_bicycle_model_sqp_hessian_f_uu {

using namespace PythonMath;
using namespace PythonNumpy;

using State_Hessian_uu_Type_SparseAvailable = SparseAvailable<
    ColumnAvailable<false, false>, ColumnAvailable<false, false>,
    ColumnAvailable<false, false>, ColumnAvailable<false, false>,
    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
    ColumnAvailable<true, true>, ColumnAvailable<true, true>>;

using State_Hessian_uu_Type =
    SparseMatrix_Type<float, State_Hessian_uu_Type_SparseAvailable>;

template <typename X_Type, typename U_Type, typename Parameter_Type>
class Function {
public:
  static inline auto sympy_function(const float q0, const float v,
                                    const float delta, const float delta_time,
                                    const float wheel_base, const float q3)
      -> State_Hessian_uu_Type {

    State_Hessian_uu_Type result;

    float x0 = tan(delta);

    float x1 = v * x0;

    float x2 = static_cast<float>(1) / wheel_base;

    float x3 = delta_time * x2 / static_cast<float>(2);

    float x4 = x1 * x3;

    float x5 = cos(x4);

    float x6 = q0 * x5;

    float x7 = x0 * x0;

    float x8 = delta_time * delta_time;

    float x9 = static_cast<float>(1) / wheel_base / wheel_base;

    float x10 = x8 * x9 / static_cast<float>(4);

    float x11 = x10 * x7;

    float x12 = sin(x4);

    float x13 = x7 + static_cast<float>(1);

    float x14 = x13 * x3;

    float x15 = q0 * x12;

    float x16 = q3 * x5;

    float x17 = x10 * x6;

    float x18 = x1 * x13;

    float x19 = q3 * v * x0 * x12 * x13 * x8 * x9 / 4 - x14 * x15 - x14 * x16 -
                x17 * x18;

    float x20 = static_cast<float>(2) * x7 + static_cast<float>(2);

    float x21 = x20 * x4;

    float x22 = v * v;

    float x23 = x13 * x13;

    float x24 = x22 * x23;

    float x25 = q3 * x12;

    float x26 = x10 * x18;

    float x27 = delta_time * q0 * x13 * x2 * x5 / static_cast<float>(2) -
                x14 * x25 - x15 * x26 - x16 * x26;

    float x28 = x10 * x24;

    result.template set<0, 0>(static_cast<float>(0));
    result.template set<0, 1>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(0));
    result.template set<1, 1>(static_cast<float>(0));
    result.template set<2, 0>(static_cast<float>(0));
    result.template set<2, 1>(static_cast<float>(0));
    result.template set<3, 0>(static_cast<float>(0));
    result.template set<3, 1>(static_cast<float>(0));
    result.template set<4, 0>(static_cast<float>(
        q3 * x12 * x7 * x8 * x9 / static_cast<float>(4) - x11 * x6));
    result.template set<4, 1>(static_cast<float>(x19));
    result.template set<5, 0>(static_cast<float>(x19));
    result.template set<5, 1>(static_cast<float>(
        q3 * x12 * x22 * x23 * x8 * x9 / static_cast<float>(4) - x15 * x21 -
        x16 * x21 - x17 * x24));
    result.template set<6, 0>(static_cast<float>(-q0 * x11 * x12 - x11 * x16));
    result.template set<6, 1>(static_cast<float>(x27));
    result.template set<7, 0>(static_cast<float>(x27));
    result.template set<7, 1>(static_cast<float>(
        delta_time * q0 * v * x0 * x2 * x20 * x5 / static_cast<float>(2) -
        x15 * x28 - x16 * x28 - x21 * x25));

    return result;
  }

  static inline auto function(const X_Type X, const U_Type U,
                              const Parameter_Type Parameters)
      -> State_Hessian_uu_Type {

    float q0 = X.template get<2, 0>();

    float q3 = X.template get<3, 0>();

    float v = U.template get<0, 0>();

    float delta = U.template get<1, 0>();

    float delta_time = Parameters.delta_time;

    float wheel_base = Parameters.wheel_base;

    return sympy_function(q0, v, delta, delta_time, wheel_base, q3);
  }
};

} // namespace kinematic_bicycle_model_sqp_hessian_f_uu

#endif // __KINEMATIC_BICYCLE_MODEL_SQP_HESSIAN_F_UU_HPP__
