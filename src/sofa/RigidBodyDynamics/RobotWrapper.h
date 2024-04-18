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

#include <sofa/RigidBodyDynamics/config.h>

#include <sofa/core/objectmodel/BaseObject.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace sofa::rigidbodydynamics
{

  using namespace core::objectmodel;

  class SOFA_RIGIDBODYDYNAMICS_API RobotWrapper : public BaseObject
  {
  public:
    SOFA_CLASS(RobotWrapper, BaseObject);

    void init() override;

    void reinit() override;

    void reset() override;

    const std::shared_ptr<pinocchio::Model>& model() const
    {
        return m_model;
    }
    const std::shared_ptr<pinocchio::GeometryModel>& collisionModel() const
    {
        return m_collision_model;
    }
    const std::shared_ptr<pinocchio::GeometryModel>& visualModel() const
    {
        return m_visual_model;
    }

  private:
    RobotWrapper();

    std::shared_ptr<pinocchio::Model> m_model;
    std::shared_ptr<pinocchio::GeometryModel> m_collision_model;
    std::shared_ptr<pinocchio::GeometryModel> m_visual_model;
    // bool canLoad();

    // bool load();
  };

} /// namespace sofa::rigidbodydynamics