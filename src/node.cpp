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


#include <ros/ros.h>
#include <comau_ik_solver/comau_kin.h>
#include <comau_ik_solver/comau_ik_solver.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh("~");

  comau::ParallelogramIk ik;

  Eigen::Quaterniond q(0.224, 0.932, 0.129, 0.255);

  Eigen::Affine3d T06;

  tf::TransformListener listener_;

  tf::StampedTransform location_transform;
  ros::Time t0 = ros::Time(0);
  std::string tf_error;

  std::string a_name="base_link";
  std::string b_name="flange";

  if (!listener_.waitForTransform(a_name,
                                  b_name,
                                  t0,
                                  ros::Duration(10),
                                  ros::Duration(0.01),
                                  &tf_error))
  {
    ROS_WARN("Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(),tf_error.c_str());
    return 0;
  }

  try
  {
    listener_.lookupTransform(a_name, b_name, t0, location_transform);
  }
  catch (...)
  {
    ROS_WARN("Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(),tf_error.c_str());
    return 0;
  }

  tf::poseTFToEigen(location_transform,T06);

  std::array<std::array<double,6>,8> sol=ik.comauIk(T06);
  for (std::array<double,6>& q: sol)
  {
    assert( std::abs((T06.inverse()*ik.comauFk(q)).matrix().norm()-2)<1e-6);
    for (int idx=0;idx<6;idx++)
      std::cout << "- J"<<idx<<": "<<q[idx] <<", ";
    std::cout<<std::endl;
  }
  return 0;
}
