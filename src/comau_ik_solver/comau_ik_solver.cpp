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
#include <stdexcept>

#include <ik_solver/internal/utils.h>

#if ROS_X == 1
  #include <pluginlib/class_list_macros.h>
#elif ROS_X == 2
  #include <pluginlib/class_list_macros.hpp>
#else
  #error "No ROS version defined"
#endif

#if ROS_X == 1
  #define LOG_ERROR(x) ROS_ERROR_STREAM(x)
#elif ROS_X == 2
  #define IKLOGGER rclcpp::get_logger("comau_ik_solver")
  #define LOG_ERROR(x) RCLCPP_ERROR_STREAM(IKLOGGER, x)
#endif

PLUGINLIB_EXPORT_CLASS(ik_solver::ComauIkSolver, ik_solver::IkSolver)

namespace ik_solver
{

bool ComauIkSolver::config(const std::string& param_ns)
{
  if(!IkSolver::config(param_ns))
  {
    return false;
  }

  if (base_frame_.find("base_link") == std::string::npos)
  {
    LOG_ERROR(param_ns << "/base_frame should be set equal to [PREFIX]base_link instead of " << base_frame_);
    return false;
  }
  std::string prefix = base_frame_;
  std::string to_erase = "base_link";

  size_t pos = std::string::npos;
  while ((pos = prefix.find(to_erase)) != std::string::npos)
  {
    // If found then erase it from string
    prefix.erase(pos, to_erase.length());
  }

  if (flange_frame_.find("flange") == std::string::npos)
  {
    LOG_ERROR(param_ns << "/flange_frame should be set equal to [PREFIX]tool0 instead of " << flange_frame_);
    return false;
  }

  std::string what;
  double gamma_min_deg = 32;
  cnr::param::get(param_ns + "gamma_min_deg", gamma_min_deg, what);
  gamma_min_ = gamma_min_deg * M_PI / 180.0;

  double epsilon_min_deg = 32;
  cnr::param::get(param_ns + "epsilon_min_deg", epsilon_min_deg, what);
  epsilon_min_ = epsilon_min_deg * M_PI / 180.0;

#if defined (COMAU_NJ_GENERIC)
  urdf::JointConstSharedPtr j;
  j = model_->getJoint( "joint_1");
  auto z1 = j->parent_to_joint_origin_transform.position.z;

  j = model_->getJoint( "joint_2");
  auto x2 = j->parent_to_joint_origin_transform.position.x;
  auto z2 = j->parent_to_joint_origin_transform.position.z;

  j = model_->getJoint( "joint_2m");
  auto z3 = j->parent_to_joint_origin_transform.position.z;

  j = model_->getJoint( "joint_4");
  auto x4 = j->parent_to_joint_origin_transform.position.x;
  auto z4 = j->parent_to_joint_origin_transform.position.z;

  j = model_->getJoint( "joint_5");
  auto x5 = j->parent_to_joint_origin_transform.position.x;

  j = model_->getJoint( "joint_6");
  auto x6 = j->parent_to_joint_origin_transform.position.x;
  ik_.reset(new comau::ParallelogramIk(z1, x2,z2,z3,x4,z4,x5,x6));
#else
  ik_.reset(new comau::ParallelogramIk());
#endif
  return true;
}

Solutions ComauIkSolver::getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds,
                                      const int& desired_solutions, const int& min_stall_iterations, const int& max_stall_iterations)
{
  Solutions ret;
  Configurations q_sols;

  std::array<std::array<double, N_JOINTS>, N_SOLS> sol = ik_->comauIk(T_base_flange, gamma_min_, epsilon_min_);

  for (std::array<double, N_JOINTS>& q : sol)
  {
    Eigen::VectorXd solution(N_JOINTS);
    // for (size_t idx = 0; idx < 6; idx++)
    // {
    //   solution(idx) = q[idx];
    // }
    solution = Eigen::VectorXd::Map(&q[0], q.size());

    auto out_of_bound = ik_solver::outOfBound(solution, this->ub_, this->lb_);

    if (out_of_bound.size())
    {
      continue;
    }

    if(solution.hasNaN())
    {
      std::cout << "solution:"<< solution.transpose() << std::endl;
      assert(0);
    }

    q_sols.push_back(solution);
  }

  auto sols = ik_solver::getMultiplicity(q_sols,this->ub_, this->lb_, this->revolute_);
  
  ret.configurations() = sols;
  ret.translation_residuals().resize(sols.size(),0.0);
  ret.rotation_residuals().resize(sols.size(),0.0);
  
  return ret;
}

Eigen::Affine3d ComauIkSolver::getFK(const Eigen::VectorXd& s)
{
  std::array<double, N_JOINTS> q;
  for (size_t idx = 0; idx < N_JOINTS; idx++)
    q.at(idx) = s(idx);
  return ik_->comauFk(q);
}

}  // end namespace ik_solver
