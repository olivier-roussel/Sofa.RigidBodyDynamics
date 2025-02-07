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

#include <coal/shape/geometric_shapes.h>
#include <pinocchio/spatial/se3.hpp>
#include <sofa/component/topology/container/constant/MeshTopology.h>

namespace sofa::rigidbodydynamics
{
  sofa::component::topology::container::constant::MeshTopology::SPtr
  boxToMeshTopology(const std::shared_ptr<coal::Box>& geom,
    const pinocchio::SE3& tf,
    const Eigen::Vector3d& scale);

  sofa::component::topology::container::constant::MeshTopology::SPtr
  cylinderToMeshTopology(const std::shared_ptr<coal::Cylinder>& geom,
    const pinocchio::SE3& tf,
    const Eigen::Vector3d& scale,
    unsigned int resolution);

  sofa::component::topology::container::constant::MeshTopology::SPtr
  sphereToMeshTopology(const std::shared_ptr<coal::Sphere>& geom,
    const pinocchio::SE3& tf,
    const Eigen::Vector3d& scale,
    unsigned int numStacks,
    unsigned int numSlices);

} // namespace sofa::rigidbodydynamics
