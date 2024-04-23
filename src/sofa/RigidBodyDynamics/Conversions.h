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
  template <typename Scalar>
  inline sofa::type::Quat<Scalar> quatToSofaType(const Eigen::Quaternion<Scalar> &in_quat)
  {
    return sofa::type::Quat<Scalar>(in_quat.x(), in_quat.y(), in_quat.z(), in_quat.w());
  }

  template <typename Scalar>
  inline sofa::type::Vec<3, Scalar> vec3ToSofaType(const Eigen::Vector3<Scalar> &in_vec)
  {
    return sofa::type::Vec<3, Scalar>(in_vec.x(), in_vec.y(), in_vec.z());
  }

  template <typename Scalar>
  inline sofa::defaulttype::RigidCoord<3, Scalar> se3ToSofaType(const pinocchio::SE3Tpl<Scalar> &in_pose)
  {
    return sofa::defaulttype::RigidCoord<3, Scalar>(vec3ToSofaType(in_pose.translation()), quatToSofaType(Eigen::Quaternion<Scalar>{in_pose.rotation()}));
  }

  template <typename Vector6Like>
  inline sofa::defaulttype::RigidDeriv<3, typename Vector6Like::Scalar> vec6ToSofaType(const Vector6Like &in_vel)
  {
    return sofa::defaulttype::RigidDeriv<3, typename Vector6Like::Scalar>(vec3ToSofaType<typename Vector6Like::Scalar>(in_vel.template head<3>()), vec3ToSofaType<typename Vector6Like::Scalar>(in_vel.template head<3>()));
  }

  template <typename VectorVec1Type>
  inline Eigen::VectorXd vectorVec1ToEigen(const VectorVec1Type& in_v, size_t size)
  {
    assert(in_v.size() >= size);

    Eigen::VectorXd out(size);
    for(auto i = 0ul; i < size; ++i)
    {
      out[i] = in_v[i](0);
    }
    return out;
  }

  template <typename VectorScalarType>
  inline Eigen::VectorXd vectorToEigen(const VectorScalarType& in_v, size_t size)
  {
    assert(in_v.size() >= size);

    Eigen::VectorXd out(size);
    for(auto i = 0ul; i < size; ++i)
    {
      out[i] = in_v[i];
    }
    return out;
  }
} // namespace sofa::rigidbodydynamics
