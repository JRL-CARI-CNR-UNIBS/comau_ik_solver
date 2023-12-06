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

#ifndef COMAU_IK_SOLVER__COMAU_KIN_H
#define COMAU_IK_SOLVER__COMAU_KIN_H


#include <ik_solver_core/ik_solver_base_class.h>
#include <Eigen/Geometry>



namespace comau
{

struct COMAU_NJ_PARAMS
{
  COMAU_NJ_PARAMS() = default;
  virtual ~COMAU_NJ_PARAMS() = default;
  double z1;
  double x2;
  double z2;
  double z3;
  double x4;
  double z4;
  double x5;
  double x6;
};

struct COMAU_NJ_370_27_PARAMS : COMAU_NJ_PARAMS
{
  COMAU_NJ_370_27_PARAMS() = default;
  virtual ~COMAU_NJ_370_27_PARAMS() = default;
  constexpr static const double z1 = 0.44800000000;
  constexpr static const double x2 = 0.46000000000;
  constexpr static const double z2 = 0.69200000000;
  constexpr static const double z3 = 1.05000000000;
  constexpr static const double x4 = 0.15500000000;
  constexpr static const double z4 = 0.25000000000;
  constexpr static const double x5 = 1.05000000000;
  constexpr static const double x6 = 0.28200000000;
};

struct COMAU_NJ_220_27_PARAMS : COMAU_NJ_PARAMS
{
  COMAU_NJ_220_27_PARAMS() = default;
  virtual ~COMAU_NJ_220_27_PARAMS() = default;
  constexpr static const double z1 = 0.326000000;
  constexpr static const double x2 = 0.400000000;
  constexpr static const double z2 = 0.504000000;
  constexpr static const double z3 = 1.175000000;
  constexpr static const double x4 = 0.106000000;
  constexpr static const double z4 = 0.250000000;
  constexpr static const double x5 = 1.023300000;
  constexpr static const double x6 = 0.22700000;
};


class ParallelogramIk
{
public:

  const COMAU_NJ_PARAMS p_;
  
  ParallelogramIk() = delete;
  ParallelogramIk(const COMAU_NJ_PARAMS& params);
  virtual ~ParallelogramIk() = default;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
  ASSUMPTION: PARALLEOGRAM is  RECTANGLE
  Parallogram Angle in proxmimity of the robot shoulder : gamma
  Parallogram Angle in proxmimity of the robot elbow    : epsilon

  We have two limits:
    - gamma_min
    - epsilon_min

  alpha = angle between the active crank and the horizontal axis
  beta  = angle between link3 and the horizontal axis

  gamma = pi - (alpha + beta) -> the angle of the parallelogram at corner the bottom is the complement to PI of alpha and beta
  epsilon = (alpha+beta) -> the angle of the parallelogram at the top corner  is the complement to PI of gamma

  alpha = -q3 -pi/2
  beta = -q2 + pi/2

  gamma > gamma_min     ==>  pi - (alpha + beta) > gamma_min  ==>   pi - (-q2-q3) > gamma_min ==> q2+q3 > gamma_min - pi
  epsilon > epsilon_min ==> (alpha + beta) > epsilon_min      ==> (-q2-q3) > epsilon_min      ==> q2+q3 < -epsilon_min
  */
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::array<std::array<double, 6>, 8> comauIk(const Eigen::Affine3d& T06, double gamma_min, double epsilon_min) const;
  Eigen::Affine3d comauFk(std::array<double, 6>& q) const;

  bool checkQ2Q3(double q2, double q3, double gamma_min, double epsilon_min) const;


protected:
  Eigen::Affine3d T0_j1;
  // Eigen::Affine3d Tj1_1;
  Eigen::Affine3d T1_j2;
  //Eigen::Affine3d Tj2_2;
  Eigen::Affine3d T2_j3;
  //Eigen::Affine3d Tj3_3;
  Eigen::Affine3d T3_j4;
  //Eigen::Affine3d Tj4_4;
  Eigen::Affine3d T4_j5;
  //Eigen::Affine3d Tj5_5;
  Eigen::Affine3d T5_j6;
  //Eigen::Affine3d Tj6_6;
  Eigen::Affine3d T6_f;

  void computeQ2Q3(const double& pj2_5x, const double& pj2_5z, double& q2_1, double& q3_1, double& q2_2, double& q3_2) const;

  std::array<std::array<double, 3>, 2> comauWrist(const Eigen::Matrix3d& R36) const;

  Eigen::Affine3d computeFK03(const double& q1, const double& q2, const double& q3) const;
};

}  // namespace comau


#endif  // COMAU_IK_SOLVER__COMAU_KIN_H