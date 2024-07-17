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
  // TODO use namespaces for explicit naming instead
  template <typename Scalar>
  inline sofa::type::Quat<Scalar> quatToSofaType(const Eigen::Quaternion<Scalar> &in_quat)
  {
    return sofa::type::Quat<Scalar>(in_quat.x(), in_quat.y(), in_quat.z(), in_quat.w());
  }

  template <typename Scalar>
  inline Eigen::Quaternion<Scalar> quatToEigen(const sofa::type::Quat<Scalar> &in_quat)
  {
    return Eigen::Quaternion<Scalar>(in_quat[0], in_quat[1], in_quat[2], in_quat[3]);
  }

  template <typename Scalar>
  inline sofa::type::Vec<3, Scalar> vec3ToSofaType(const Eigen::Vector3<Scalar> &in_vec)
  {
    return sofa::type::Vec<3, Scalar>(in_vec.x(), in_vec.y(), in_vec.z());
  }

  template <typename Scalar>
  inline Eigen::Vector3<Scalar> vec3ToEigen(const sofa::type::Vec<3, Scalar> &in_vec)
  {
    return Eigen::Vector3<Scalar>(in_vec.x(), in_vec.y(), in_vec.z());
  }

  template <typename Scalar>
  inline sofa::defaulttype::RigidCoord<3, Scalar> se3ToSofaType(const pinocchio::SE3Tpl<Scalar> &in_pose)
  {
    return sofa::defaulttype::RigidCoord<3, Scalar>(vec3ToSofaType(in_pose.translation()), quatToSofaType(Eigen::Quaternion<Scalar>{in_pose.rotation()}));
  }

  template <typename Scalar>
  inline Eigen::Matrix<Scalar, 7, 1> se3ToEigen(const sofa::defaulttype::RigidCoord<3, Scalar> &in_pose)
  {
    Eigen::Matrix<Scalar, 7, 1> out;
    const auto& v = in_pose.getCenter();
    const auto& q = in_pose.getOrientation();
    out << v.x(), v.y(), v.z(), q[0], q[1], q[2], q[3];
    return out;
  }

  template <typename Scalar>
  inline pinocchio::SE3Tpl<Scalar> se3ToPinocchio(const sofa::defaulttype::RigidCoord<3, Scalar> &in_pose)
  {
    return pinocchio::SE3Tpl<Scalar>(vec3ToEigen(in_pose.getCenter()), quatToEigen(Eigen::Quaternion<Scalar>{in_pose.getOrientation()}));
  }

  template <typename Vector6Like>
  inline sofa::defaulttype::RigidDeriv<3, typename Vector6Like::Scalar> vec6ToSofaType(const Vector6Like &in_vel)
  {
    return sofa::defaulttype::RigidDeriv<3, typename Vector6Like::Scalar>(vec3ToSofaType<typename Vector6Like::Scalar>(in_vel.template head<3>()), vec3ToSofaType<typename Vector6Like::Scalar>(in_vel.template tail<3>()));
  }

   template <typename Scalar>
  inline Eigen::Matrix<Scalar, 6, 1> spatialVelocityToEigen(const sofa::defaulttype::RigidDeriv<3, Scalar> &in_vel)
  {
    Eigen::Matrix<Scalar, 6, 1> out;
    const auto& v = in_vel.getVCenter();
    const auto& q = in_vel.getVOrientation();
    out << v.x(), v.y(), v.z(), q[0], q[1], q[2], q[3];
    return out;
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

  template <typename Scalar>
  inline sofa::type::Mat<3, 3, Scalar> mat3ToSofaType(const Eigen::Matrix3<Scalar>& in_m)
  {
    sofa::type::Mat<3, 3, Scalar> out;
    for(auto i = 0ul; i < 3; ++i)
    {
      for(auto j = 0ul; j < 3; ++j)
      {
        out(i, j) = in_m(i, j);
      }
    }
    return out;
  }
} // namespace sofa::rigidbodydynamics
