#ifndef __KINEMATIC_BICYCLE_MODEL_EKF_A_HPP__
#define __KINEMATIC_BICYCLE_MODEL_EKF_A_HPP__

#include "python_numpy.hpp"

namespace kinematic_bicycle_model_ekf_A {

using namespace PythonNumpy;

using SparseAvailable_ekf_A =
    SparseAvailable<ColumnAvailable<true, false, true, false>,
                    ColumnAvailable<false, true, true, true>,
                    ColumnAvailable<false, false, true, true>,
                    ColumnAvailable<false, false, true, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ekf_A>;

inline auto make(void) -> type {

  return make_SparseMatrix<SparseAvailable_ekf_A>(
      static_cast<float>(1.0), static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0), static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0));
}

} // namespace kinematic_bicycle_model_ekf_A

#endif // __KINEMATIC_BICYCLE_MODEL_EKF_A_HPP__
