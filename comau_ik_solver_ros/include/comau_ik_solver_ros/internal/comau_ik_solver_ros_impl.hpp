#ifndef COMAU_IK_SOLVER__COMAU_IK_SOLVER_ROS__INTERNAL__COMAU_IK_SOLVER_ROS_IMPL_H
#define COMAU_IK_SOLVER__COMAU_IK_SOLVER_ROS__INTERNAL__COMAU_IK_SOLVER_ROS_IMPL_H

#include <urdf/model.h>
#include <ros/param.h>
#include <comau_ik_solver/comau_ik_solver.h>
#include <comau_ik_solver_ros/comau_ik_solver_ros.h>
#include "comau_ik_solver/comau_kin.h"

namespace ik_solver 
{

template<typename P>
inline bool ComauIkSolverConfigurator<P>::get_configuration(IkSolverOptionsPtr opts, const ros::NodeHandle& nh, const std::string& params_ns, std::string& what)
{
  auto _opts = dynamic_cast<ComauIkSolverOptions<P>*>(opts);
  double gamma_min_deg = 32;
  ros::param::get(params_ns + "gamma_min_deg", gamma_min_deg);
  _opts->gamma_min_ = gamma_min_deg * M_PI / 180.0;

  double epsilon_min_deg = 32;
  ros::param::get(params_ns + "epsilon_min_deg", epsilon_min_deg);
  _opts->epsilon_min_ = epsilon_min_deg * M_PI / 180.0;

  if(std::is_same<P, comau::COMAU_NJ_PARAMS>::value)
  { 
    urdf::Model model;
    model.initParam("robot_description");
    urdf::JointConstSharedPtr j;
    j = model.getJoint( "joint_1");
    _opts->z1 = j->parent_to_joint_origin_transform.position.z;

    j = model.getJoint( "joint_2");
    _opts->x2 = j->parent_to_joint_origin_transform.position.x;
    _opts->z2 = j->parent_to_joint_origin_transform.position.z;

    j = model.getJoint( "joint_2m");
    _opts->z3 = j->parent_to_joint_origin_transform.position.z;

    j = model.getJoint( "joint_4");
    _opts->x4 = j->parent_to_joint_origin_transform.position.x;
    _opts->z4 = j->parent_to_joint_origin_transform.position.z;

    j = model.getJoint( "joint_5");
    _opts->x5 = j->parent_to_joint_origin_transform.position.x;

    j = model.getJoint( "joint_6");
    _opts->x6 = j->parent_to_joint_origin_transform.position.x;
  }
}

};


}

#endif // COMAU_IK_SOLVER__COMAU_IK_SOLVER_ROS__INTERNAL__COMAU_IK_SOLVER_ROS_IMPL_H