#ifndef COMAU_IK_SOLVER__COMAU_IK_SOLVER_ROS__COMAU_IK_SOLVER_ROS__H
#define COMAU_IK_SOLVER__COMAU_IK_SOLVER_ROS__COMAU_IK_SOLVER_ROS__H

#include <ros/param.h>
#include <comau_ik_solver/comau_kin.h>
#include <ik_solver_ros/ik_solver_ros.h>

namespace ik_solver 
{

template<typename P>
struct ComauIkSolverConfigurator  : IkSolverConfigurator
{
  ComauIkSolverConfigurator() = default;
  ~ComauIkSolverConfigurator() = default;

  ComauIkSolverConfigurator(const ComauIkSolverConfigurator& ) = delete;
  ComauIkSolverConfigurator(ComauIkSolverConfigurator&& ) = delete;
  ComauIkSolverConfigurator& operator=(const ComauIkSolverConfigurator& rhs ) = delete;

  virtual bool get_configuration(IkSolverOptionsPtr opts, const ros::NodeHandle& nh, const std::string& params_ns, std::string& what) override;

};

using NJ220_27_IkSolverConfigurator = ComauIkSolverConfigurator<comau::COMAU_NJ_220_27_PARAMS>;
using NJ370_27_IkSolverConfigurator = ComauIkSolverConfigurator<comau::COMAU_NJ_370_27_PARAMS>;
using NJ_IkSolverConfigurator       = ComauIkSolverConfigurator<comau::COMAU_NJ_PARAMS>;

}

#include <comau_ik_solver_ros/internal/comau_ik_solver_ros_impl.hpp>

#endif // COMAU_IK_SOLVER__COMAU_IK_SOLVER_ROS__COMAU_IK_SOLVER_ROS__H