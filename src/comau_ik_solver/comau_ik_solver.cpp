/*
Copyright (c) 2022, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <comau_ik_solver/comau_ik_solver.h>
#include <comau_ik_solver/comau_kin.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ik_solver::ComauIkSolver, ik_solver::IkSolver)


namespace ik_solver
{

bool ComauIkSolver::customConfig()
{
  if (base_frame_.find("base_link") == std::string::npos)
  {
      ROS_ERROR("%s/base_frame should be set equal to [PREFIX]base_link instead of %s",nh_.getNamespace().c_str(),base_frame_.c_str());
      return false;
  }
  std::string prefix=base_frame_;
  std::string to_erase="base_link";

  size_t pos = std::string::npos;
  while ((pos  = prefix.find(to_erase) )!= std::string::npos)
  {
    // If found then erase it from string
    prefix.erase(pos, to_erase.length());
  }

  if (flange_frame_.find("flange") == std::string::npos)
  {
      ROS_ERROR("%s/flange_frame should be set equal to [PREFIX]tool0 instead of %s",nh_.getNamespace().c_str(),flange_frame_.c_str());
      return false;
  }


  urdf::JointConstSharedPtr j;
  j=model_.getJoint(prefix+"joint_1");
  ik.z1=j->parent_to_joint_origin_transform.position.z;

  j=model_.getJoint(prefix+"joint_2");
  ik.x2=j->parent_to_joint_origin_transform.position.x;
  ik.z2=j->parent_to_joint_origin_transform.position.z;

  j=model_.getJoint(prefix+"joint_2m");
  ik.z3=j->parent_to_joint_origin_transform.position.z;

  j=model_.getJoint(prefix+"joint_4");
  ik.x4=j->parent_to_joint_origin_transform.position.x;
  ik.z4=j->parent_to_joint_origin_transform.position.z;

  j=model_.getJoint(prefix+"joint_5");
  ik.x5=j->parent_to_joint_origin_transform.position.x;

  j=model_.getJoint(prefix+"joint_6");
  ik.x6=j->parent_to_joint_origin_transform.position.x;

  return true;
}


std::vector<Eigen::VectorXd> ComauIkSolver::getIk(const Eigen::Affine3d& T_base_flange,
                                                   const std::vector<Eigen::VectorXd> & seeds,
                                                   const int& desired_solutions,
                                                   const int& max_stall_iterations)
{
  std::vector<Eigen::VectorXd> q_sols;

  std::array<std::array<double,6>,8> sol=ik.comauIk(T_base_flange);
  for (std::array<double,6>& q: sol)
  {

    Eigen::VectorXd solution(6);
    bool out_of_bound=false;
    for (size_t idx=0;idx<6;idx++)
    {
      if (std::isnan(q[idx]))
      {
        out_of_bound=true;
        break;
      }
      else if (q[idx]>ub_(idx))
      {
        out_of_bound=true;
        break;
      }
      else if (q[idx]<lb_(idx))
      {
        out_of_bound=true;
        break;
      }
      solution(idx)=q[idx];
    }
    if (out_of_bound)
      continue;
    q_sols.push_back(solution);
  }

  return getMultiplicity(q_sols);
}



}   // end namespace ik_solver
