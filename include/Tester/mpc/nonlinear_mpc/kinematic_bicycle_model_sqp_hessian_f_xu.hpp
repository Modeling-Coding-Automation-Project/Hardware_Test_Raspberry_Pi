#ifndef __KINEMATIC_BICYCLE_MODEL_SQP_HESSIAN_F_XU_HPP__
#define __KINEMATIC_BICYCLE_MODEL_SQP_HESSIAN_F_XU_HPP__

#include "python_math.hpp"
#include "python_numpy.hpp"

namespace kinematic_bicycle_model_sqp_hessian_f_xu {

using namespace PythonMath;
using namespace PythonNumpy;

using State_Hessian_xu_Type_SparseAvailable = SparseAvailable<
    ColumnAvailable<false, false>, ColumnAvailable<false, false>,
    ColumnAvailable<true, false>, ColumnAvailable<false, false>,
    ColumnAvailable<false, false>, ColumnAvailable<false, false>,
    ColumnAvailable<true, false>, ColumnAvailable<true, false>,
    ColumnAvailable<false, false>, ColumnAvailable<false, false>,
    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
    ColumnAvailable<false, false>, ColumnAvailable<false, false>,
    ColumnAvailable<true, true>, ColumnAvailable<true, true>>;

using State_Hessian_xu_Type =
    SparseMatrix_Type<float, State_Hessian_xu_Type_SparseAvailable>;

template <typename X_Type, typename U_Type, typename Parameter_Type>
class Function {
public:
  static inline auto sympy_function(const float q0, const float v,
                                    const float delta, const float delta_time,
                                    const float wheel_base, const float q3)
      -> State_Hessian_xu_Type {

    State_Hessian_xu_Type result;

    float x0 = delta_time * q0;

    float x1 = static_cast<float>(1) / wheel_base;

    float x2 = tan(delta);

    float x3 = delta_time * x1 * x2 / static_cast<float>(2);

    float x4 = v * x3;

    float x5 = sin(x4);

    float x6 = -x3 * x5;

    float x7 = delta_time * v * x1 * (x2 * x2 + static_cast<float>(1)) /
               static_cast<float>(2);

    float x8 = -x5 * x7;

    float x9 = cos(x4);

    float x10 = x3 * x9;

    float x11 = x7 * x9;

    result.template set<0, 0>(static_cast<float>(0));
    result.template set<0, 1>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(0));
    result.template set<1, 1>(static_cast<float>(0));
    result.template set<2, 0>(static_cast<float>(static_cast<float>(4) * x0));
    result.template set<2, 1>(static_cast<float>(0));
    result.template set<3, 0>(static_cast<float>(0));
    result.template set<3, 1>(static_cast<float>(0));
    result.template set<4, 0>(static_cast<float>(0));
    result.template set<4, 1>(static_cast<float>(0));
    result.template set<5, 0>(static_cast<float>(0));
    result.template set<5, 1>(static_cast<float>(0));
    result.template set<6, 0>(
        static_cast<float>(static_cast<float>(2) * delta_time * q3));
    result.template set<6, 1>(static_cast<float>(0));
    result.template set<7, 0>(static_cast<float>(static_cast<float>(2) * x0));
    result.template set<7, 1>(static_cast<float>(0));
    result.template set<8, 0>(static_cast<float>(0));
    result.template set<8, 1>(static_cast<float>(0));
    result.template set<9, 0>(static_cast<float>(0));
    result.template set<9, 1>(static_cast<float>(0));
    result.template set<10, 0>(static_cast<float>(x6));
    result.template set<10, 1>(static_cast<float>(x8));
    result.template set<11, 0>(static_cast<float>(-x10));
    result.template set<11, 1>(static_cast<float>(-x11));
    result.template set<12, 0>(static_cast<float>(0));
    result.template set<12, 1>(static_cast<float>(0));
    result.template set<13, 0>(static_cast<float>(0));
    result.template set<13, 1>(static_cast<float>(0));
    result.template set<14, 0>(static_cast<float>(x10));
    result.template set<14, 1>(static_cast<float>(x11));
    result.template set<15, 0>(static_cast<float>(x6));
    result.template set<15, 1>(static_cast<float>(x8));

    return result;
  }

  static inline auto function(const X_Type X, const U_Type U,
                              const Parameter_Type Parameters)
      -> State_Hessian_xu_Type {

    float q0 = X.template get<2, 0>();

    float q3 = X.template get<3, 0>();

    float v = U.template get<0, 0>();

    float delta = U.template get<1, 0>();

    float delta_time = Parameters.delta_time;

    float wheel_base = Parameters.wheel_base;

    return sympy_function(q0, v, delta, delta_time, wheel_base, q3);
  }
};

} // namespace kinematic_bicycle_model_sqp_hessian_f_xu

#endif // __KINEMATIC_BICYCLE_MODEL_SQP_HESSIAN_F_XU_HPP__
