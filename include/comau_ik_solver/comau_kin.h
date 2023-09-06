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

#include <ik_solver/ik_solver_base_class.h>
#include <Eigen/Geometry>
// #define TOLERANCE 1e-3
namespace comau
{
class ParallelogramIk
{
public:
  ParallelogramIk();

  double z1;
  double x2;
  double z2;
  double z3;
  double x4;
  double z4;
  double x5;
  double x6;

public:
  std::array<std::array<double, 6>, 8> comauIk(const Eigen::Affine3d& T06);
  Eigen::Affine3d comauFk(std::array<double, 6>& q);

protected:
  Eigen::Affine3d T0_j1;
  Eigen::Affine3d Tj1_1;
  Eigen::Affine3d T1_j2;
  Eigen::Affine3d Tj2_2;
  Eigen::Affine3d T2_j3;
  Eigen::Affine3d Tj3_3;
  Eigen::Affine3d T3_j4;
  Eigen::Affine3d Tj4_4;
  Eigen::Affine3d T4_j5;
  Eigen::Affine3d Tj5_5;
  Eigen::Affine3d T5_j6;
  Eigen::Affine3d Tj6_6;
  Eigen::Affine3d T6_f;

  void computeQ2Q3(const double& pj2_5x, const double& pj2_5z, double& q2_1, double& q3_1, double& q2_2, double& q3_2);

  std::array<std::array<double, 3>, 2> comauWrist(const Eigen::Matrix3d& R36);

  Eigen::Affine3d computeFK03(const double& q1, const double& q2, const double& q3);
};

}  // namespace comau
