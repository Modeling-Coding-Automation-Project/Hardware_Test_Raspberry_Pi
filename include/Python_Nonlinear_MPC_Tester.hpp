#ifndef __PYTHON_NONLINEAR_MPC_TESTER_HPP__
#define __PYTHON_NONLINEAR_MPC_TESTER_HPP__

#include "python_mpc.hpp"

#include "kinematic_bicycle_model_nonlinear_mpc.hpp"
#include "kinematic_bicycle_model_nonlinear_mpc_ekf_parameter.hpp"
#include "nonlinear_mpc_reference_path.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>

namespace nonlinear_mpc_namespace = kinematic_bicycle_model_nonlinear_mpc;
namespace state_function =
    kinematic_bicycle_model_nonlinear_mpc_ekf_state_function;
namespace measurement_function =
    kinematic_bicycle_model_nonlinear_mpc_ekf_measurement_function;

using Parameter_Type =
    kinematic_bicycle_model_nonlinear_mpc_ekf_parameter::Parameter_Type;
using ReferenceTrajectory_Type =
    kinematic_bicycle_model_nonlinear_mpc::ReferenceTrajectory_Type;

using Tester_MPC_Type = kinematic_bicycle_model_nonlinear_mpc::type;

class Python_Nonlinear_MPC_Tester {
public:
  /* Constant, Type */
  static constexpr std::size_t INPUT_SIZE = nonlinear_mpc_namespace::INPUT_SIZE;
  static constexpr std::size_t STATE_SIZE = nonlinear_mpc_namespace::STATE_SIZE;
  static constexpr std::size_t OUTPUT_SIZE =
      nonlinear_mpc_namespace::OUTPUT_SIZE;

  static constexpr std::size_t NP = nonlinear_mpc_namespace::NP;

public:
  /* Constructor */
  Python_Nonlinear_MPC_Tester();

  /* Destructor */
  ~Python_Nonlinear_MPC_Tester();

public:
  /* Functions */
  void test_mpc(void);

private:
  /* Variables */
  std::unique_ptr<Tester_MPC_Type> _mpc;
};

#endif // __PYTHON_NONLINEAR_MPC_TESTER_HPP__
