#ifndef __KINEMATIC_BICYCLE_MODEL_SQP_HESSIAN_F_UX_HPP__
#define __KINEMATIC_BICYCLE_MODEL_SQP_HESSIAN_F_UX_HPP__

#include "python_math.hpp"
#include "python_numpy.hpp"

namespace kinematic_bicycle_model_sqp_hessian_f_ux {

using namespace PythonMath;
using namespace PythonNumpy;

using State_Hessian_ux_Type_SparseAvailable =
    SparseAvailable<ColumnAvailable<false, false, true, false>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, true, true>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, true, true>,
                    ColumnAvailable<false, false, true, true>,
                    ColumnAvailable<false, false, true, true>,
                    ColumnAvailable<false, false, true, true>>;

using State_Hessian_ux_Type =
    SparseMatrix_Type<float, State_Hessian_ux_Type_SparseAvailable>;

template <typename X_Type, typename U_Type, typename Parameter_Type>
class Function {
public:
  static inline auto sympy_function(const float q0, const float v,
                                    const float delta, const float delta_time,
                                    const float wheel_base, const float q3)
      -> State_Hessian_ux_Type {

    State_Hessian_ux_Type result;

    float x0 = delta_time * q0;

    float x1 = static_cast<float>(1) / wheel_base;

    float x2 = tan(delta);

    float x3 = delta_time * x1 * x2 / static_cast<float>(2);

    float x4 = v * x3;

    float x5 = sin(x4);

    float x6 = -x3 * x5;

    float x7 = cos(x4);

    float x8 = x3 * x7;

    float x9 = delta_time * v * x1 * (x2 * x2 + static_cast<float>(1)) /
               static_cast<float>(2);

    float x10 = -x5 * x9;

    float x11 = x7 * x9;

    result.template set<0, 0>(static_cast<float>(0));
    result.template set<0, 1>(static_cast<float>(0));
    result.template set<0, 2>(static_cast<float>(static_cast<float>(4) * x0));
    result.template set<0, 3>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(0));
    result.template set<1, 1>(static_cast<float>(0));
    result.template set<1, 2>(static_cast<float>(0));
    result.template set<1, 3>(static_cast<float>(0));
    result.template set<2, 0>(static_cast<float>(0));
    result.template set<2, 1>(static_cast<float>(0));
    result.template set<2, 2>(
        static_cast<float>(static_cast<float>(2) * delta_time * q3));
    result.template set<2, 3>(static_cast<float>(static_cast<float>(2) * x0));
    result.template set<3, 0>(static_cast<float>(0));
    result.template set<3, 1>(static_cast<float>(0));
    result.template set<3, 2>(static_cast<float>(0));
    result.template set<3, 3>(static_cast<float>(0));
    result.template set<4, 0>(static_cast<float>(0));
    result.template set<4, 1>(static_cast<float>(0));
    result.template set<4, 2>(static_cast<float>(x6));
    result.template set<4, 3>(static_cast<float>(-x8));
    result.template set<5, 0>(static_cast<float>(0));
    result.template set<5, 1>(static_cast<float>(0));
    result.template set<5, 2>(static_cast<float>(x10));
    result.template set<5, 3>(static_cast<float>(-x11));
    result.template set<6, 0>(static_cast<float>(0));
    result.template set<6, 1>(static_cast<float>(0));
    result.template set<6, 2>(static_cast<float>(x8));
    result.template set<6, 3>(static_cast<float>(x6));
    result.template set<7, 0>(static_cast<float>(0));
    result.template set<7, 1>(static_cast<float>(0));
    result.template set<7, 2>(static_cast<float>(x11));
    result.template set<7, 3>(static_cast<float>(x10));

    return result;
  }

  static inline auto function(const X_Type X, const U_Type U,
                              const Parameter_Type Parameters)
      -> State_Hessian_ux_Type {

    float q0 = X.template get<2, 0>();

    float q3 = X.template get<3, 0>();

    float v = U.template get<0, 0>();

    float delta = U.template get<1, 0>();

    float delta_time = Parameters.delta_time;

    float wheel_base = Parameters.wheel_base;

    return sympy_function(q0, v, delta, delta_time, wheel_base, q3);
  }
};

} // namespace kinematic_bicycle_model_sqp_hessian_f_ux

#endif // __KINEMATIC_BICYCLE_MODEL_SQP_HESSIAN_F_UX_HPP__
