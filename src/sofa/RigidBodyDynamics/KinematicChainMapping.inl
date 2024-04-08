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
#include <sofa/RigidBodyDynamics/KinematicChainMapping.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/visual/DrawTool.h>

#include <sofa/core/objectmodel/BaseContext.h>
// #include <sofa/component/topology/container/constant/MeshTopology.h>

#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>

using namespace sofa::defaulttype;

namespace sofa::component::mapping
{
  template <class TIn, class TInRoot, class TOut>
  KinematicChainMapping<TIn, TInRoot, TOut>::KinematicChainMapping()
      : d_modelDir(initData(&d_modelDir, std::string{}, "modelDir", "Directory containing robot models")), d_urdfFile(initData(&d_urdfFile, std::string{}, "urdfFile", "URDF file absolute path")), m_model{nullptr}, m_collisionModel{nullptr}, m_visualModel{nullptr}
  {
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::init()
  {
    msg_info() << "========= KinematicChainMapping init";
    d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);

    if (this->getFromModels1().empty())
    {
      msg_error() << "While iniatilizing ; input Model not found.";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    if (this->getToModels().empty())
    {
      msg_error() << "While iniatilizing ; output Model not found.";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    Inherit::init();
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::bwdInit()
  {
    msg_info() << "========= KinematicChainMapping bwdInit";
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::reset()
  {
    msg_info() << "========= KinematicChainMapping reset";

    init();
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::apply(
      const core::MechanicalParams *mparams, const type::vector<OutDataVecCoord *> &dataVecOutPos,
      const type::vector<const InDataVecCoord *> &dataVecInPos,
      const type::vector<const InRootDataVecCoord *> &dataVecInRootPos)
  {
    SOFA_UNUSED(mparams);

    assert(dataVecInPos.size() == 1);     // one vector of 1-size dofs
    assert(dataVecInRootPos.size() <= 1); // one or zero free floating root dof

    assert(m_model);
    assert(m_collisionModel);
    assert(m_visualModel);

    msg_info() << "========= KinematicChainMapping apply";
    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    // Create data required by the algorithms
    pinocchio::Data data(*m_model);
    pinocchio::GeometryData collision_data(*m_collisionModel);
    pinocchio::GeometryData visual_data(*m_visualModel);

    // map in configuration to pinocchio
    Eigen::VectorXd q_in = Eigen::VectorXd::Zero(m_model->nq);
    InVecCoord in_dofs = dataVecInPos[0]->getValue();
    // XXX add helpers to convert types ?
    for (auto i = 0ul; i < in_dofs.size(); ++i)
      q_in[i] = in_dofs[i](0);
    msg_info() << "q_in: " << q_in.transpose();

    // Perform the forward kinematics over the kinematic tree
    pinocchio::forwardKinematics(*m_model, data, q_in);
    msg_info() << " fwd kinematics computed";

    // Update Geometry models
    pinocchio::updateGeometryPlacements(*m_model, data, *m_collisionModel, collision_data);
    pinocchio::updateGeometryPlacements(*m_model, data, *m_visualModel, visual_data);
    msg_info() << " geometry placements updated";

    msg_info() << " Robot data oMi vector size: " << data.oMi.size();
    
    assert(data.oMi.size() == out.size());

    for (auto jointIdx = 0ul; jointIdx < dataVecOutPos.size(); ++jointIdx)
    {
      OutDataVecCoord *bodyPoseW = dataVecOutPos[jointIdx];
      
      assert(bodyPoseW->getValue().size() == 1);

      helper::WriteAccessor<OutDataVecCoord> accessBodyPoseW(bodyPoseW);
      const auto in_quat = Eigen::Quaterniond{data.oMi[jointIdx].rotation()};
      sofa::type::Quat out_quat(in_quat.x(), in_quat.y(), in_quat.z(), in_quat.w());
      const auto &in_pos = data.oMi[jointIdx].translation();
      sofa::type::Vec3 out_pos(in_pos.x(), in_pos.y(), in_pos.z());
      accessBodyPoseW[0] = Rigid3dTypes::Coord(out_pos, out_quat);
    }

  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJ(
      const core::MechanicalParams *mparams, const type::vector<OutDataVecDeriv *> &dataVecOutVel,
      const type::vector<const InDataVecDeriv *> &dataVecInVel,
      const type::vector<const InRootDataVecDeriv *> &dataVecInRootVel)
  {
    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;
    
    // TODO
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJT(
      const core::MechanicalParams *mparams, const type::vector<InDataVecDeriv *> &dataVecOut1Force,
      const type::vector<InRootDataVecDeriv *> &dataVecOut2Force,
      const type::vector<const OutDataVecDeriv *> &dataVecInForce)
  {
    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    // TODO
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::draw(const core::visual::VisualParams *vparams)
  {
    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    // TODO
  }
} // namespace sofa::component::mapping
