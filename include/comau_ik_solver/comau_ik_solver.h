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

#pragma once

#include <memory>
#include <ik_solver/ik_solver.hpp>
#include <comau_ik_solver/comau_kin.h>

// #define TOLERANCE 1e-3
namespace ik_solver
{
class ComauIkSolver : public IkSolver
{
public:
  virtual bool config(const std::string& param_ns = "") override;
  virtual Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds, const int& desired_solutions = -1, const int& min_stall_iterations = -1, const int& max_stall_iterations = -1) override;

  virtual Eigen::Affine3d getFK(const Eigen::VectorXd& s) override;

protected:
  
  double gamma_min_;
  double epsilon_min_;
  
  constexpr static unsigned int N_JOINTS {6};
  constexpr static unsigned int N_SOLS    {8};

  std::shared_ptr<comau::ParallelogramIk> ik_;
};
}  //  namespace ik_solver
