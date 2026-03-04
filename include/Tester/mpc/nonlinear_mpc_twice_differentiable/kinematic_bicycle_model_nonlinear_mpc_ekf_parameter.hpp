#ifndef __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_PARAMETER_HPP__
#define __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_PARAMETER_HPP__

namespace kinematic_bicycle_model_nonlinear_mpc_ekf_parameter {

class Parameter {
public:
  float wheel_base = static_cast<float>(2.8);
  float delta_time = static_cast<float>(0.1);
};

using Parameter_Type = Parameter;

} // namespace kinematic_bicycle_model_nonlinear_mpc_ekf_parameter

#endif // __KINEMATIC_BICYCLE_MODEL_NONLINEAR_MPC_EKF_PARAMETER_HPP__
