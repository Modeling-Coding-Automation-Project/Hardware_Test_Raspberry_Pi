#ifndef KINEMATIC_BICYCLE_MODEL_OP_EN_PARAMETER_HPP_
#define KINEMATIC_BICYCLE_MODEL_OP_EN_PARAMETER_HPP_

namespace kinematic_bicycle_model_op_en_parameter {

class Parameter {
public:
  float wheel_base = static_cast<float>(2.8);
  float delta_time = static_cast<float>(0.1);
};

} // namespace kinematic_bicycle_model_op_en_parameter

#endif // KINEMATIC_BICYCLE_MODEL_OP_EN_PARAMETER_HPP_
