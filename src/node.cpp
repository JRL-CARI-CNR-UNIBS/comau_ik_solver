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

#include <Eigen/Geometry>
#include "Eigen/src/Core/Matrix.h"
#include "ros/node_handle.h"
#include <ros/ros.h>

#include <tf_conversions/tf_eigen.h>

#include <ik_solver/internal/utils.h>

#include <comau_ik_solver/comau_kin.h>
#include <comau_ik_solver/comau_ik_solver.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh("~");

  #if defined(COMAU_NJ_370_27) || defined (COMAU_NJ_220_27)
    comau::ParallelogramIk ik;
  #elif defined(COMAU_NJ_GENERIC)
    urdf::Model model;
    model.initParam("robot_description");
    urdf::JointConstSharedPtr j;
    j = model.getJoint( "joint_1");
    auto z1 = j->parent_to_joint_origin_transform.position.z;

    j = model.getJoint( "joint_2");
    auto x2 = j->parent_to_joint_origin_transform.position.x;
    auto z2 = j->parent_to_joint_origin_transform.position.z;

    j = model.getJoint( "joint_2m");
    auto z3 = j->parent_to_joint_origin_transform.position.z;

    j = model.getJoint( "joint_4");
    auto x4 = j->parent_to_joint_origin_transform.position.x;
    auto z4 = j->parent_to_joint_origin_transform.position.z;

    j = model.getJoint( "joint_5");
    auto x5 = j->parent_to_joint_origin_transform.position.x;

    j = model.getJoint( "joint_6");
    auto x6 = j->parent_to_joint_origin_transform.position.x;
    
    comau::ParallelogramIk ik(z1,x2,z2,z3,x4,z4,x5,x6);
  #endif
  std::cout << "=== TEST 1 ======================================" << std::endl;
  {

    Eigen::Affine3d T06;

    tf::TransformListener listener_;

    tf::StampedTransform location_transform;
    ros::Time t0 = ros::Time(0);
    std::string tf_error;

    std::string a_name = "base_link";
    std::string b_name = "flange";

    if (!listener_.waitForTransform(a_name, b_name, t0, ros::Duration(10), ros::Duration(0.01), &tf_error))
    {
      ROS_WARN("Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(),
               tf_error.c_str());
      return 0;
    }

    try
    {
      listener_.lookupTransform(a_name, b_name, t0, location_transform);
    }
    catch (...)
    {
      ROS_WARN("Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(),
               tf_error.c_str());
      return 0;
    }

    tf::poseTFToEigen(location_transform, T06);

    std::cout << "T06:\n" << T06.matrix() << std::endl;
    std::array<std::array<double, 6>, 8> sol = ik.comauIk(T06, 45*M_PI/180.0, 30*M_PI/180.0);
    for (std::array<double, 6>& q : sol)
    {
      assert(std::abs((T06.inverse() * ik.comauFk(q)).matrix().norm() - 2) < 1e-6);
      for (int idx = 0; idx < 6; idx++)
        std::cout << "- J" << idx << ": " << q[idx] << ", ";
      std::cout << std::endl;
    }
  }

  std::cout << "=== TEST 2 ======================================" << std::endl;
  {
    Eigen::Affine3d T06;
    T06.translation() << 2.15097, 1.62142, 0.727435;

    T06.linear().matrix() <<  -0.986432, -0.0742417,  -0.146425,    
 -0.069868,   0.996949, -0.0347971,    
  0.148561, -0.0240946,   -0.98861;   
       

    ik_solver::ComauIkSolver comau_ik_solver;
    comau_ik_solver.config(ros::NodeHandle("~"), "/tapping_ik_solver/mounted_robot_ik/");

    std::cout << "T06:\n" << T06.matrix() << std::endl;
    auto sol = ik.comauIk(T06,45*M_PI/180.0, 30*M_PI/180.0);
    for (std::array<double, 6>& q : sol)
    {
      Eigen::VectorXd qq(6);
      qq = Eigen::VectorXd::Map(&q[0], q.size());

      std::cout << "IK/FK error: " << (T06.inverse() * ik.comauFk(q)).matrix().determinant() - 1.0 << "\t";

      auto vv = ik_solver::outOfBound(qq, comau_ik_solver.ub(), comau_ik_solver.lb());
      std::cout << "Axes out of Bound: ";
      for (const auto v : vv)
      {
        std::cout << v << ",";
      }
      std::cout << "\t[" << qq.transpose() << "]" << std::endl;
    }
  }

  return 0;
}
