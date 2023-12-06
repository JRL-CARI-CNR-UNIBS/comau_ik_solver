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

#ifndef COMAU_IK_SOLVER__COMAU_IK_SOLVER_H
#define COMAU_IK_SOLVER__COMAU_IK_SOLVER_H

#include <ik_solver_core/ik_solver_base_class.h>
#include <comau_ik_solver/comau_kin.h>
#include <memory>

// #define TOLERANCE 1e-3
namespace ik_solver
{

template<typename P>
struct ComauIkSolverOptions : public IkSolverOptions
{
  double gamma_min_;
  double epsilon_min_;
  const unsigned int n_joints = 6;
  const unsigned int n_sol = 8;
  P params_;

  using Ptr = std::shared_ptr<ComauIkSolverOptions>;
  using ConstPtr = std::shared_ptr<const ComauIkSolverOptions>;
};


template<typename P>
class ComauIkSolver : public IkSolver
{
public:
  virtual bool config(IkSolverOptionsConstPtr nh, std::string& what) override;
  virtual Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds, const int& desired_solutions = -1, const int& min_stall_iterations = -1, const int& max_stall_iterations = -1) override;

  virtual Eigen::Affine3d getFK(const Eigen::VectorXd& s) override;

private: 

  typename ComauIkSolverOptions<P>::ConstPtr opts_;

  std::shared_ptr<comau::ParallelogramIk> ik_;
};

using NJ220_27_IkSolver = ComauIkSolver<comau::COMAU_NJ_220_27_PARAMS>;
using NJ370_27_IkSolver = ComauIkSolver<comau::COMAU_NJ_370_27_PARAMS>;
using NJ_IkSolver       = ComauIkSolver<comau::COMAU_NJ_PARAMS>;

template<typename P> using ComauIkSolverOptionsPtr = typename ComauIkSolverOptions<P>::Ptr;
template<typename P> using ComauIkSolverOptionsConstPtr = typename ComauIkSolverOptions<P>::ConstPtr;

using NJ220_27_IkSolverOptions = ComauIkSolverOptions<comau::COMAU_NJ_220_27_PARAMS>;
using NJ370_27_IkSolverOptions = ComauIkSolverOptions<comau::COMAU_NJ_370_27_PARAMS>;
using NJ_IkSolverOptions       = ComauIkSolverOptions<comau::COMAU_NJ_PARAMS>;

using NJ220_27_IkSolverOptionsPtr = ComauIkSolverOptions<comau::COMAU_NJ_220_27_PARAMS>::Ptr;
using NJ370_27_IkSolverOptionsPtr = ComauIkSolverOptions<comau::COMAU_NJ_370_27_PARAMS>::Ptr;
using NJ_IkSolverOptionsPtr       = ComauIkSolverOptions<comau::COMAU_NJ_PARAMS>::Ptr;

using NJ220_27_IkSolverOptionsConstPtr = ComauIkSolverOptions<comau::COMAU_NJ_220_27_PARAMS>::ConstPtr;
using NJ370_27_IkSolverOptionsConstPtr = ComauIkSolverOptions<comau::COMAU_NJ_370_27_PARAMS>::ConstPtr;
using NJ_IkSolverOptionsConstPtr       = ComauIkSolverOptions<comau::COMAU_NJ_PARAMS>::ConstPtr;

}  //  namespace ik_solver

#include <comau_ik_solver/internal/comau_ik_solver_impl.hpp>

#endif