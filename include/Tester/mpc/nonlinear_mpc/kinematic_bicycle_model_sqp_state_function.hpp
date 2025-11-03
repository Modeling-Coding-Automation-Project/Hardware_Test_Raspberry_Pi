#ifndef __KINEMATIC_BICYCLE_MODEL_SQP_STATE_FUNCTION_HPP__
#define __KINEMATIC_BICYCLE_MODEL_SQP_STATE_FUNCTION_HPP__

#include "python_math.hpp"

namespace kinematic_bicycle_model_sqp_state_function {

using namespace PythonMath;

template <typename X_Type, typename U_Type, typename Parameter_Type>
class Function {
public:
  static inline auto sympy_function(const float q0, const float v,
                                    const float py, const float delta,
                                    const float delta_time, const float px,
                                    const float wheel_base, const float q3)
      -> X_Type {

    X_Type result;

    float x0 = delta_time * v;

    float x1 = x0 * tan(delta) / (2 * wheel_base);

    float x2 = cos(x1);

    float x3 = sin(x1);

    result.template set<0, 0>(
        static_cast<float>(px + x0 * (2 * (q0 * q0) - 1)));
    result.template set<1, 0>(static_cast<float>(py + 2 * q0 * q3 * x0));
    result.template set<2, 0>(static_cast<float>(q0 * x2 - q3 * x3));
    result.template set<3, 0>(static_cast<float>(q0 * x3 + q3 * x2));

    return result;
  }

  static inline auto function(const X_Type X, const U_Type U,
                              const Parameter_Type Parameters) -> X_Type {

    float px = X.template get<0, 0>();

    float py = X.template get<1, 0>();

    float q0 = X.template get<2, 0>();

    float q3 = X.template get<3, 0>();

    float v = U.template get<0, 0>();

    float delta = U.template get<1, 0>();

    float delta_time = Parameters.delta_time;

    float wheel_base = Parameters.wheel_base;

    return sympy_function(q0, v, py, delta, delta_time, px, wheel_base, q3);
  }
};

} // namespace kinematic_bicycle_model_sqp_state_function

#endif // __KINEMATIC_BICYCLE_MODEL_SQP_STATE_FUNCTION_HPP__
