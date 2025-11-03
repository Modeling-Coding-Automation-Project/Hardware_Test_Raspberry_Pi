#ifndef __KINEMATIC_BICYCLE_MODEL_EKF_C_HPP__
#define __KINEMATIC_BICYCLE_MODEL_EKF_C_HPP__

#include "python_numpy.hpp"

namespace kinematic_bicycle_model_ekf_C {

using namespace PythonNumpy;

using type = DiagMatrix_Type<float, 4>;

inline auto make(void) -> type {

  return make_DiagMatrix<4>(static_cast<float>(1.0), static_cast<float>(1.0),
                            static_cast<float>(1.0), static_cast<float>(1.0));
}

} // namespace kinematic_bicycle_model_ekf_C

#endif // __KINEMATIC_BICYCLE_MODEL_EKF_C_HPP__
