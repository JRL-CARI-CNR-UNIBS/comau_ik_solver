/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#include <comau_ik_solver/comau_kin.h>
#include "Eigen/src/Geometry/Transform.h"

namespace std
{
template<int N>
inline std::string to_string(const std::array<double, N>& aa)
{
  std::string ret = "[";
  for(size_t i=0;i<aa.size(); i++)
  {
    const auto & a= aa.at(i);
    std::stringstream ss; 
    ss.precision(4);
    ss << std::fixed <<  a << (i==aa.size()-1?"]" : ",");
    ret += ss.str();
  }
  return ret;
}
template<int N, int M>
inline std::string to_string(const std::array<std::array<double, N>,M>& aa)
{
  std::string ret = "[";
  for(size_t i=0;i<aa.size(); i++)
  {
    const auto & a= aa.at(i);
    std::stringstream ss; 
    ss.precision(4);
    ss << std::fixed << " " << std::to_string<N>(a) << (i==aa.size()-1?"]" : ",\n");
    ret += ss.str();
  }
  return ret;
}
}

namespace comau
{

#if defined(COMAU_NJ_370_27)
  const double ParallelogramIk::z1 = 0.44800000000;
  const double ParallelogramIk::x2 = 0.46000000000;
  const double ParallelogramIk::z2 = 0.69200000000;
  const double ParallelogramIk::z3 = 1.05000000000;
  const double ParallelogramIk::x4 = 0.15500000000;
  const double ParallelogramIk::z4 = 0.25000000000;
  const double ParallelogramIk::x5 = 1.05000000000;
  const double ParallelogramIk::x6 = 0.28200000000;

#elif defined(COMAU_NJ_220_27)

  const double ParallelogramIk::z1 = 0.326000000;
  const double ParallelogramIk::x2 = 0.400000000;
  const double ParallelogramIk::z2 = 0.504000000;
  const double ParallelogramIk::z3 = 1.175000000;
  const double ParallelogramIk::x4 = 0.106000000;
  const double ParallelogramIk::z4 = 0.250000000;
  const double ParallelogramIk::x5 = 1.023300000;
  const double ParallelogramIk::x6 = 0.22700000;

#endif

#if defined(COMAU_NJ_220_27) || defined(COMAU_NJ_370_27)
ParallelogramIk::ParallelogramIk()
{
  T0_j1.setIdentity();
  // Tj1_1.setIdentity();
  T1_j2.setIdentity();
  // Tj2_2.setIdentity();
  T2_j3.setIdentity();
  // Tj3_3.setIdentity();
  T3_j4.setIdentity();
  // Tj4_4.setIdentity();
  T4_j5.setIdentity();
  // Tj5_5.setIdentity();
  T5_j6.setIdentity();
  // Tj6_6.setIdentity();
  T6_f.setIdentity();

  T0_j1.translation()(2) = z1;

  T1_j2.translation()(0) = x2;
  T1_j2.translation()(2) = z2;

  T2_j3.translation()(2) = z3;

  T3_j4.translation()(0) = x4;
  T3_j4.translation()(2) = z4;

  T4_j5.translation()(0) = x5;

  T5_j6.translation()(0) = x6;

  T6_f = Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitY());
}
#elif defined(COMAU_NJ_GENERIC)
ParallelogramIk::ParallelogramIk(const double _z1, const double _x2,const double _z2,const double _z3,const double _x4,const double _z4,const double _x5,const double _x6)
: z1(_z1), x2(_x2),z2(_z2),z3(_z3),x4(_x4),z4(_z4),x5(_x5),x6(_x6)
{

  T0_j1.setIdentity();
  // Tj1_1.setIdentity();
  T1_j2.setIdentity();
  // Tj2_2.setIdentity();
  T2_j3.setIdentity();
  // Tj3_3.setIdentity();
  T3_j4.setIdentity();
  // Tj4_4.setIdentity();
  T4_j5.setIdentity();
  // Tj5_5.setIdentity();
  T5_j6.setIdentity();
  // Tj6_6.setIdentity();
  T6_f.setIdentity();

  T0_j1.translation()(2) = z1;

  T1_j2.translation()(0) = x2;
  T1_j2.translation()(2) = z2;

  T2_j3.translation()(2) = z3;

  T3_j4.translation()(0) = x4;
  T3_j4.translation()(2) = z4;

  T4_j5.translation()(0) = x5;

  T5_j6.translation()(0) = x6;

  T6_f = Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitY());
}
#endif

bool ParallelogramIk::checkQ2Q3(double q2, double q3, double gamma_min, double epsilon_min) const
{
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
  return ((q2+q3) > std::fabs(gamma_min) - M_PI) && ((q2+q3) < -std::fabs(epsilon_min));

}

void ParallelogramIk::computeQ2Q3(const double& pj2_5x, const double& pj2_5z, double& q2_1, double& q3_1, double& q2_2,
                                  double& q3_2) const
{
  double delta = (pj2_5x * pj2_5x) * (x4 * x4) * 2.0 + (pj2_5x * pj2_5x) * (x5 * x5) * 2.0 +
                 (pj2_5z * pj2_5z) * (x4 * x4) * 2.0 + (pj2_5z * pj2_5z) * (x5 * x5) * 2.0 +
                 (pj2_5x * pj2_5x) * (z3 * z3) * 2.0 + (pj2_5x * pj2_5x) * (z4 * z4) * 2.0 +
                 (pj2_5z * pj2_5z) * (z3 * z3) * 2.0 + (pj2_5z * pj2_5z) * (z4 * z4) * 2.0 -
                 (x4 * x4) * (x5 * x5) * 6.0 + (x4 * x4) * (z3 * z3) * 2.0 - (x4 * x4) * (z4 * z4) * 2.0 +
                 (x5 * x5) * (z3 * z3) * 2.0 - (x5 * x5) * (z4 * z4) * 2.0 + (z3 * z3) * (z4 * z4) * 2.0 -
                 x4 * (x5 * x5 * x5) * 4.0 - (x4 * x4 * x4) * x5 * 4.0 - pj2_5x * pj2_5x * pj2_5x * pj2_5x -
                 pj2_5z * pj2_5z * pj2_5z * pj2_5z - x4 * x4 * x4 * x4 - x5 * x5 * x5 * x5 - z3 * z3 * z3 * z3 -
                 z4 * z4 * z4 * z4 - (pj2_5x * pj2_5x) * (pj2_5z * pj2_5z) * 2.0 + (pj2_5x * pj2_5x) * x4 * x5 * 4.0 +
                 (pj2_5z * pj2_5z) * x4 * x5 * 4.0 + x4 * x5 * (z3 * z3) * 4.0 - x4 * x5 * (z4 * z4) * 4.0;

  if (delta < 0)
  {
    q2_1 = q3_1 = q2_2 = q2_2 = std::nan("1");
    return;
  }

  double den_q2 =
      pj2_5z * z3 * 2.0 - x4 * x5 * 2.0 + pj2_5x * pj2_5x + pj2_5z * pj2_5z - x4 * x4 - x5 * x5 + z3 * z3 - z4 * z4;
  double den_q3 = pj2_5z * x4 * 2.0 + pj2_5z * x5 * 2.0 - pj2_5x * z4 * 2.0 + x4 * x5 * 2.0 + pj2_5x * pj2_5x +
                  pj2_5z * pj2_5z + x4 * x4 + x5 * x5 - z3 * z3 + z4 * z4;

  q2_1 = std::atan2(pj2_5x * z3 * 2.0 + sqrt(delta), den_q2) * 2.0;
  q2_2 = std::atan2(pj2_5x * z3 * 2.0 - sqrt(delta), den_q2) * 2.0;
  q3_1 = std::atan2(pj2_5x * x4 * 2.0 + pj2_5x * x5 * 2.0 + pj2_5z * z4 * 2.0 - sqrt(delta), den_q3) * -2.0;
  q3_2 = std::atan2(pj2_5x * x4 * 2.0 + pj2_5x * x5 * 2.0 + pj2_5z * z4 * 2.0 + sqrt(delta), den_q3) * -2.0;

  assert(std::abs(pj2_5x - (-z4 * cos(q3_1) - x4 * sin(q3_1) - x5 * sin(q3_1) + z3 * sin(q2_1))) < 1e-6);
  assert(std::abs(pj2_5z - (x4 * cos(q3_1) + x5 * cos(q3_1) + z3 * cos(q2_1) - z4 * sin(q3_1))) < 1e-6);
  assert(std::abs(pj2_5x - (-z4 * cos(q3_2) - x4 * sin(q3_2) - x5 * sin(q3_2) + z3 * sin(q2_2))) < 1e-6);
  assert(std::abs(pj2_5z - (x4 * cos(q3_2) + x5 * cos(q3_2) + z3 * cos(q2_2) - z4 * sin(q3_2))) < 1e-6);
}

std::array<std::array<double, 3>, 2> ParallelogramIk::comauWrist(const Eigen::Matrix3d& R36) const
{
  double q4_1, q5_1, q6_1;
  double q4_2, q5_2, q6_2;
  q5_1 = std::acos(R36(0, 2));
  q5_2 = -q5_1;

  if (std::sin(q5_1) > 0)
  {
    q4_1 = std::atan2(-R36(1, 2), -R36(2, 2));
    q6_1 = std::atan2(-R36(0, 1), -R36(0, 0));
  }
  else
  {
    q4_1 = std::atan2(R36(1, 2), R36(2, 2));
    q6_1 = std::atan2(R36(0, 1), R36(0, 0));
  }

  if (sin(q5_2) > 0)
  {
    q4_2 = std::atan2(-R36(1, 2), -R36(2, 2));
    q6_2 = std::atan2(-R36(0, 1), -R36(0, 0));
  }
  else
  {
    q4_2 = std::atan2(R36(1, 2), R36(2, 2));
    q6_2 = std::atan2(R36(0, 1), R36(0, 0));
  }

  std::array<double, 3> sol1;
  std::array<double, 3> sol2;
  sol1.at(0) = q4_1;
  sol1.at(1) = q5_1;
  sol1.at(2) = q6_1;
  sol2.at(0) = q4_2;
  sol2.at(1) = q5_2;
  sol2.at(2) = q6_2;
  std::array<std::array<double, 3>, 2> sol;
  sol.at(0) = sol1;
  sol.at(1) = sol2;
  return sol;
}

Eigen::Affine3d ParallelogramIk::computeFK03(const double& q1, const double& q2, const double& q3) const
{
  auto Tj1_1 = Eigen::AngleAxisd(-q1, Eigen::Vector3d::UnitZ());
  auto Tj2_2 = Eigen::AngleAxisd(q2, Eigen::Vector3d::UnitY());
  auto Tj3_3 = Eigen::AngleAxisd(-M_PI * 0.5 - q2 - q3, Eigen::Vector3d::UnitY());

  Eigen::Affine3d T03 = T0_j1 * Tj1_1 * T1_j2 * Tj2_2 * T2_j3 * Tj3_3 * T3_j4 * T4_j5;

  return T03;
}

std::array<std::array<double, 6>, 8> ParallelogramIk::comauIk(const Eigen::Affine3d& T06, double gamma_min, double epsilon_min) const
{
  
  std::array<std::array<double, 6>, 8> solutions{};

  Eigen::Vector3d p05 = T06.translation() - x6 * T06.linear() * Eigen::Vector3d::UnitZ();

  double q1a = std::atan2(-p05(1), p05(0));
  double q1b = std::atan2(p05(1), -p05(0));

  double q2_1, q2_2, q3_1, q3_2;
  double q1, q2, q3;

  // case A
  q1 = q1a;
  auto Tj1_1 = Eigen::AngleAxisd(-q1, Eigen::Vector3d::UnitZ());
  Eigen::Affine3d T0_j2 = T0_j1 * Tj1_1 * T1_j2;

  Eigen::Vector3d pj2_5 = T0_j2.inverse() * p05;

  computeQ2Q3(pj2_5(0), pj2_5(2), q2_1, q3_1, q2_2, q3_2);
  if(checkQ2Q3(q2_1, q3_1, gamma_min, epsilon_min))
  {
    q2 = q2_1;
    q3 = q3_1;
    Eigen::Affine3d T03 = computeFK03(q1, q2, q3);

    Eigen::Affine3d T36 = T03.inverse() * T06;
    std::array<std::array<double, 3>, 2> q456 = comauWrist(T36.linear().matrix());
    solutions[0][0] = q1;
    solutions[0][1] = q2;
    solutions[0][2] = q3;
    solutions[0][3] = q456[0][0];
    solutions[0][4] = q456[0][1];
    solutions[0][5] = q456[0][2];

    solutions[1][0] = q1;
    solutions[1][1] = q2;
    solutions[1][2] = q3;
    solutions[1][3] = q456[1][0];
    solutions[1][4] = q456[1][1];
    solutions[1][5] = q456[1][2];
  }
  else
  {
    std::fill(std::begin(solutions.at(0)), std::end(solutions.at(0)), std::nan("1"));
    std::fill(std::begin(solutions.at(1)), std::end(solutions.at(1)), std::nan("1"));
  }

  q1 = q1a;

  if(checkQ2Q3(q2_2, q3_2, gamma_min, epsilon_min))
  {
    q2 = q2_2;
    q3 = q3_2;
    auto T03 = computeFK03(q1, q2, q3);

    auto T36 = T03.inverse() * T06;

    auto q456 = comauWrist(T36.linear().matrix());
    auto s = std::to_string<3,2>(q456);

    solutions[2][0] = q1;
    solutions[2][1] = q2;
    solutions[2][2] = q3;
    solutions[2][3] = q456[0][0];
    solutions[2][4] = q456[0][1];
    solutions[2][5] = q456[0][2];

    solutions[3][0] = q1;
    solutions[3][1] = q2;
    solutions[3][2] = q3;
    solutions[3][3] = q456[1][0];
    solutions[3][4] = q456[1][1];
    solutions[3][5] = q456[1][2];
  }
  else
  {
    std::fill(std::begin(solutions.at(2)), std::end(solutions.at(2)), std::nan("1"));
    std::fill(std::begin(solutions.at(3)), std::end(solutions.at(3)), std::nan("1"));
  }

  // case B
  q1 = q1b;
  Tj1_1 = Eigen::AngleAxisd(-q1, Eigen::Vector3d::UnitZ());
  T0_j2 = T0_j1 * Tj1_1 * T1_j2;

  pj2_5 = T0_j2.inverse() * p05;

  computeQ2Q3(pj2_5(0), pj2_5(2), q2_1, q3_1, q2_2, q3_2);
  if(checkQ2Q3(q2_1, q3_1, gamma_min, epsilon_min))
  {
    q2 = q2_1;
    q3 = q3_1;
    auto T03 = computeFK03(q1, q2, q3);

    auto T36 = T03.inverse() * T06;
    auto q456 = comauWrist(T36.linear().matrix());
    solutions[4][0] = q1;
    solutions[4][1] = q2;
    solutions[4][2] = q3;
    solutions[4][3] = q456[0][0];
    solutions[4][4] = q456[0][1];
    solutions[4][5] = q456[0][2];

    solutions[5][0] = q1;
    solutions[5][1] = q2;
    solutions[5][2] = q3;
    solutions[5][3] = q456[1][0];
    solutions[5][4] = q456[1][1];
    solutions[5][5] = q456[1][2];
  }
  else 
  {
    std::fill(std::begin(solutions.at(4)), std::end(solutions.at(4)), std::nan("1"));
    std::fill(std::begin(solutions.at(5)), std::end(solutions.at(5)), std::nan("1"));  
  }

  q1 = q1b;
  if(checkQ2Q3(q2_1, q3_1, gamma_min, epsilon_min))
  {
    q2 = q2_2;
    q3 = q3_2;
    auto T03 = computeFK03(q1, q2, q3);

    auto T36 = T03.inverse() * T06;
    auto q456 = comauWrist(T36.linear().matrix());
    solutions[6][0] = q1;
    solutions[6][1] = q2;
    solutions[6][2] = q3;
    solutions[6][3] = q456[0][0];
    solutions[6][4] = q456[0][1];
    solutions[6][5] = q456[0][2];

    solutions[7][0] = q1;
    solutions[7][1] = q2;
    solutions[7][2] = q3;
    solutions[7][3] = q456[1][0];
    solutions[7][4] = q456[1][1];
    solutions[7][5] = q456[1][2];
  }
  else 
  {
    std::fill(std::begin(solutions.at(6)), std::end(solutions.at(6)), std::nan("1"));
    std::fill(std::begin(solutions.at(7)), std::end(solutions.at(7)), std::nan("1"));  
  }

  return solutions;
}

Eigen::Affine3d ParallelogramIk::comauFk(std::array<double, 6>& q) const
{
  auto Tj1_1 = Eigen::AngleAxisd(-q[0], Eigen::Vector3d::UnitZ());
  auto Tj2_2 = Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitY());
  auto Tj3_3 = Eigen::AngleAxisd(-M_PI * 0.5 - q[1] - q[2], Eigen::Vector3d::UnitY());
  auto Tj4_4 = Eigen::AngleAxisd(-q[3], Eigen::Vector3d::UnitX());
  auto Tj5_5 = Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitY());
  auto Tj6_6 = Eigen::AngleAxisd(-q[5], Eigen::Vector3d::UnitX());

  Eigen::Affine3d T0f =
      T0_j1 * Tj1_1 * T1_j2 * Tj2_2 * T2_j3 * Tj3_3 * T3_j4 * Tj4_4 * T4_j5 * Tj5_5 * T5_j6 * Tj6_6 * T6_f;

  return T0f;
}

}  // end namespace comau
