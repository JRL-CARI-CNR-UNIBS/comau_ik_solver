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


#ifndef COMAU_IK_SOLVER__INTERNAL__COMAU_IK_SOLVER_IMPL_H
#define COMAU_IK_SOLVER__INTERNAL__COMAU_IK_SOLVER_IMPL_H

#include <comau_ik_solver/comau_ik_solver.h>
#include <pluginlib/class_list_macros.h>
#include <ik_solver_core/utils.h>
#include <ik_solver_core/ik_solver_base_class.h>
#include <urdf_model/types.h>
#include <memory>

namespace ik_solver
{
  
template<typename P>
inline bool ComauIkSolver<P>::config(IkSolverOptionsConstPtr opts, std::string& what)
{
  if(!IkSolver::config(opts, what))
  {
    return false;
  }

  if (base_frame_.find("base_link") == std::string::npos)
  {
    what = "'base_frame' should be set equal to [PREFIX]base_link instead of "+ base_frame_+"'";
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
    what = "'flange_frame' should be set equal to [PREFIX]tool0 instead of '" +flange_frame_+"'";
    return false;
  }

  opts_ = std::dynamic_pointer_cast<const ComauIkSolverOptions<P>>(opts);

  ik_.reset(new comau::ParallelogramIk(opts_->params_));

  return true;
}

template<typename P>
inline Solutions ComauIkSolver<P>::getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds, 
                                      const int& desired_solutions, const int& min_stall_iterations, const int& max_stall_iterations)
{
  Solutions ret;
  Configurations q_sols;

  std::array<std::array<double, 6>, 8> sol = ik_->comauIk(T_base_flange, opts_->gamma_min_, opts_->epsilon_min_);

  for (std::array<double, 6>& q : sol)
  {
    Eigen::VectorXd solution(6);
    // for (size_t idx = 0; idx < 6; idx++)
    // {
    //   solution(idx) = q[idx];
    // }
    solution = Eigen::VectorXd::Map(&q[0], q.size());

    std::vector<int> out_of_bound;
    ik_solver::outOfBound(solution, this->jb_, out_of_bound);

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

  auto sols = ik_solver::getMultiplicity(q_sols,this->jb_, this->revolute_);
  
  ret.configurations() = sols;
  ret.translation_residuals().resize(sols.size(),0.0);
  ret.rotation_residuals().resize(sols.size(),0.0);
  
  return ret;
}

template<typename P>
inline Eigen::Affine3d ComauIkSolver<P>::getFK(const Eigen::VectorXd& s)
{
  std::array<double, 6> q;
  for (size_t idx = 0; idx < 6; idx++)
  {
    q.at(idx) = s(idx);
  }
  return ik_->comauFk(q);
}

}  // end namespace ik_solver

#endif // COMAU_IK_SOLVER__INTERNAL__COMAU_IK_SOLVER_IMPL_H