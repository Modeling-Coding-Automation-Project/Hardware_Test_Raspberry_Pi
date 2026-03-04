#ifndef __KINEMATIC_BICYCLE_MODEL_SQP_HESSIAN_F_XX_HPP__
#define __KINEMATIC_BICYCLE_MODEL_SQP_HESSIAN_F_XX_HPP__

#include "python_math.hpp"
#include "python_numpy.hpp"

namespace kinematic_bicycle_model_sqp_hessian_f_xx {

using namespace PythonMath;
using namespace PythonNumpy;

using State_Hessian_xx_Type_SparseAvailable =
    SparseAvailable<ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, true, false>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, false, true>,
                    ColumnAvailable<false, false, true, false>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, false, false>,
                    ColumnAvailable<false, false, false, false>>;

using State_Hessian_xx_Type =
    SparseMatrix_Type<float, State_Hessian_xx_Type_SparseAvailable>;

template <typename X_Type, typename U_Type, typename Parameter_Type>
class Function {
public:
  static inline auto sympy_function(const float delta_time, const float v)
      -> State_Hessian_xx_Type {

    State_Hessian_xx_Type result;

    float x0 = delta_time * v;

    float x1 = static_cast<float>(2) * x0;

    result.template set<0, 0>(static_cast<float>(0));
    result.template set<0, 1>(static_cast<float>(0));
    result.template set<0, 2>(static_cast<float>(0));
    result.template set<0, 3>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(0));
    result.template set<1, 1>(static_cast<float>(0));
    result.template set<1, 2>(static_cast<float>(0));
    result.template set<1, 3>(static_cast<float>(0));
    result.template set<2, 0>(static_cast<float>(0));
    result.template set<2, 1>(static_cast<float>(0));
    result.template set<2, 2>(static_cast<float>(static_cast<float>(4) * x0));
    result.template set<2, 3>(static_cast<float>(0));
    result.template set<3, 0>(static_cast<float>(0));
    result.template set<3, 1>(static_cast<float>(0));
    result.template set<3, 2>(static_cast<float>(0));
    result.template set<3, 3>(static_cast<float>(0));
    result.template set<4, 0>(static_cast<float>(0));
    result.template set<4, 1>(static_cast<float>(0));
    result.template set<4, 2>(static_cast<float>(0));
    result.template set<4, 3>(static_cast<float>(0));
    result.template set<5, 0>(static_cast<float>(0));
    result.template set<5, 1>(static_cast<float>(0));
    result.template set<5, 2>(static_cast<float>(0));
    result.template set<5, 3>(static_cast<float>(0));
    result.template set<6, 0>(static_cast<float>(0));
    result.template set<6, 1>(static_cast<float>(0));
    result.template set<6, 2>(static_cast<float>(0));
    result.template set<6, 3>(static_cast<float>(x1));
    result.template set<7, 0>(static_cast<float>(0));
    result.template set<7, 1>(static_cast<float>(0));
    result.template set<7, 2>(static_cast<float>(x1));
    result.template set<7, 3>(static_cast<float>(0));
    result.template set<8, 0>(static_cast<float>(0));
    result.template set<8, 1>(static_cast<float>(0));
    result.template set<8, 2>(static_cast<float>(0));
    result.template set<8, 3>(static_cast<float>(0));
    result.template set<9, 0>(static_cast<float>(0));
    result.template set<9, 1>(static_cast<float>(0));
    result.template set<9, 2>(static_cast<float>(0));
    result.template set<9, 3>(static_cast<float>(0));
    result.template set<10, 0>(static_cast<float>(0));
    result.template set<10, 1>(static_cast<float>(0));
    result.template set<10, 2>(static_cast<float>(0));
    result.template set<10, 3>(static_cast<float>(0));
    result.template set<11, 0>(static_cast<float>(0));
    result.template set<11, 1>(static_cast<float>(0));
    result.template set<11, 2>(static_cast<float>(0));
    result.template set<11, 3>(static_cast<float>(0));
    result.template set<12, 0>(static_cast<float>(0));
    result.template set<12, 1>(static_cast<float>(0));
    result.template set<12, 2>(static_cast<float>(0));
    result.template set<12, 3>(static_cast<float>(0));
    result.template set<13, 0>(static_cast<float>(0));
    result.template set<13, 1>(static_cast<float>(0));
    result.template set<13, 2>(static_cast<float>(0));
    result.template set<13, 3>(static_cast<float>(0));
    result.template set<14, 0>(static_cast<float>(0));
    result.template set<14, 1>(static_cast<float>(0));
    result.template set<14, 2>(static_cast<float>(0));
    result.template set<14, 3>(static_cast<float>(0));
    result.template set<15, 0>(static_cast<float>(0));
    result.template set<15, 1>(static_cast<float>(0));
    result.template set<15, 2>(static_cast<float>(0));
    result.template set<15, 3>(static_cast<float>(0));

    return result;
  }

  static inline auto function(const X_Type X, const U_Type U,
                              const Parameter_Type Parameters)
      -> State_Hessian_xx_Type {

    float v = U.template get<0, 0>();

    float delta_time = Parameters.delta_time;

    return sympy_function(delta_time, v);
  }
};

} // namespace kinematic_bicycle_model_sqp_hessian_f_xx

#endif // __KINEMATIC_BICYCLE_MODEL_SQP_HESSIAN_F_XX_HPP__
