/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#pragma once

#include <memory>
#include <ik_solver/ik_solver_base_class.h>
#include <comau_ik_solver/comau_kin.h>

// #define TOLERANCE 1e-3
namespace ik_solver
{
class ComauIkSolver : public IkSolver
{
public:
  virtual bool config(const ros::NodeHandle& nh, const std::string& param_ns = "") override;
  virtual Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds, const int& desired_solutions = -1, const int& min_stall_iterations = -1, const int& max_stall_iterations = -1) override;

  virtual Eigen::Affine3d getFK(const Eigen::VectorXd& s) override;

protected:
  
  double gamma_min_;
  double epsilon_min_;
  
  const unsigned int n_joints = 6;
  const unsigned int n_sol = 8;

  std::shared_ptr<comau::ParallelogramIk> ik_;
};
}  //  namespace ik_solver
