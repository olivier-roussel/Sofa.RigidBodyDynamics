/******************************************************************************
 *                 SOFA, Simulation Open-Framework Architecture                *
 *                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
 *                                                                             *
 * This program is free software; you can redistribute it and/or modify it     *
 * under the terms of the GNU Lesser General Public License as published by    *
 * the Free Software Foundation; either version 2.1 of the License, or (at     *
 * your option) any later version.                                             *
 *                                                                             *
 * This program is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
 * for more details.                                                           *
 *                                                                             *
 * You should have received a copy of the GNU Lesser General Public License    *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.        *
 *******************************************************************************
 * Authors: The SOFA Team and external contributors (see Authors.txt)          *
 *                                                                             *
 * Contact information: contact@sofa-framework.org                             *
 ******************************************************************************/
#pragma once

#include <Eigen/Geometry>
#include <pinocchio/spatial/se3.hpp>
#include <sofa/type/Quat.h>
#include <sofa/type/Vec.h>
#include <sofa/defaulttype/RigidCoord.h>

namespace sofa::rigidbodydynamics
{
// template <typename RealType> 
// sofa::type::Quat<RealType> toSofaType(const Eigen::Quaternion<RealType>& quat)
inline
sofa::type::Quat<double> toSofaType(const Eigen::Quaterniond& in_quat)
{
  return sofa::type::Quat<double>(in_quat.x(), in_quat.y(), in_quat.z(), in_quat.w());
}

inline
sofa::type::Vec<3, double> toSofaType(const Eigen::Vector3d& in_vec)
{
  return sofa::type::Vec<3, double>(in_vec.x(), in_vec.y(), in_vec.z());
}

inline
sofa::defaulttype::RigidCoord<3,double> toSofaType(const pinocchio::SE3& in_pose)
{
  return sofa::defaulttype::RigidCoord<3,double>(toSofaType(in_pose.translation()), toSofaType(Eigen::Quaterniond{in_pose.rotation()}));
}

} // namespace sofa::rigidbodydynamics
