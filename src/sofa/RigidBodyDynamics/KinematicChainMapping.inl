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

#include <sofa/RigidBodyDynamics/Conversions.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/visual/DrawTool.h>

#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
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

    // map in configuration to pinocchio
    InVecCoord in_dofs = dataVecInPos[0]->getValue();
    msg_info() << "in_dofs size: " << in_dofs.size() << " / model nq = " << m_model->nq;
    assert(in_dofs.size() == m_model->nv);
    Eigen::VectorXd q = sofa::rigidbodydynamics::vectorVec1ToEigen(in_dofs, m_model->nq);

    // Perform the forward kinematics over the kinematic tree
    // pinocchio::forwardKinematics(*m_model, *m_data, q_in);
    // Computes joints Jacobians and forward kinematics. Jacobians will
    // be used by applyJ and not apply function, but this is done here 
    // to avoid duplicate computation of forward kinematics
    pinocchio::computeJointJacobians(*m_model, *m_data, q);
    msg_info() << " fwd kinematics computed";

    // Update Geometry models
    pinocchio::updateGeometryPlacements(*m_model, *m_data, *m_collisionModel, *m_collisionData);
    pinocchio::updateGeometryPlacements(*m_model, *m_data, *m_visualModel, *m_visualData);
    msg_info() << " geometry placements updated";

    // msg_info() << " Robot data oMi vector size: " << m_data->oMi.size();

    // Single output vector of size njoints
    OutDataVecCoord *g_w = dataVecOutPos[0];

    assert(g_w->getValue().size() == m_data->oMi.size());

    helper::WriteAccessor<OutDataVecCoord> accessor_g_w(g_w);
    for (auto jointIdx = 0ul; jointIdx < m_model->njoints; ++jointIdx)
    {
      accessor_g_w[jointIdx] = sofa::rigidbodydynamics::se3ToSofaType(m_data->oMi[jointIdx]);
    }
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJ(
      const core::MechanicalParams *mparams, const type::vector<OutDataVecDeriv *> &dataVecOutVel,
      const type::vector<const InDataVecDeriv *> &dataVecInVel,
      const type::vector<const InRootDataVecDeriv *> &dataVecInRootVel)
  {
    msg_info() << "********* KinematicChainMapping applyJ";
    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    // map in configuration to pinocchio
    // Eigen::VectorXd dq = Eigen::VectorXd::Zero(m_model->nv);
    InVecDeriv in_dofs = dataVecInVel[0]->getValue();

    msg_info() << "in_dofs size: " << in_dofs.size() << " / model nv = " << m_model->nv;
    assert(in_dofs.size() == m_model->nv);
    Eigen::VectorXd dq = sofa::rigidbodydynamics::vectorVec1ToEigen(in_dofs, m_model->nv);

    // Single output vector of size njoints
    OutDataVecDeriv *dgdq_w = dataVecOutVel[0];

    assert(dgdq_w->getValue().size() == m_model->njoints);

    helper::WriteAccessor<OutDataVecDeriv> accessor_dgdq_w(dgdq_w);
    for (auto jointIdx = 0ul; jointIdx < m_model->njoints; ++jointIdx)
    {
      pinocchio::Data::Matrix6x J(6, m_model->nv);
      pinocchio::getJointJacobian(*m_model, *m_data, jointIdx, pinocchio::LOCAL, J);
      Eigen::VectorXd dg = J * dq;
      accessor_dgdq_w[jointIdx] = sofa::rigidbodydynamics::vec6ToSofaType<Eigen::VectorXd>(dg);
      // msg_info() << "KinematicChainMapping: setting pose: " << m_data->oMi[jointIdx] << " to joint body[" << jointIdx << "]";
    }
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJT(
      const core::MechanicalParams *mparams, const type::vector<InDataVecDeriv *> &dataVecOut1Force,
      const type::vector<InRootDataVecDeriv *> &dataVecOut2Force,
      const type::vector<const OutDataVecDeriv *> &dataVecInForce)
  {
    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    msg_info() << "********* KinematicChainMapping applyJT";

    // Single output vector of size njoints
    msg_info() << "dataVecOut1Force = " << dataVecOut1Force.size();
    msg_info() << "dataVecOut2Force = " << dataVecOut2Force.size();
    msg_info() << "dataVecInForce = " << dataVecInForce.size();

    InDataVecDeriv *dgdqT_w = dataVecOut1Force[0];
    helper::WriteAccessor<InDataVecDeriv> accessor_dgdqT_w(dgdqT_w);
    msg_info() << "dataVecOut1Force dgdqT_w size: " << accessor_dgdqT_w.size();

    const OutDataVecDeriv *wrench_w = dataVecInForce[0];
    helper::ReadAccessor<OutDataVecDeriv> accessor_wrench_w(wrench_w);
    msg_info() << "dataVecInForce accessor_wrench_w size = " << accessor_wrench_w.size();
    // assert(dgdq_w->getValue().size() == m_data->J.cols());
    for(auto i = 0ul; i < accessor_wrench_w.size(); ++i)
    {
      msg_info() << "w[" << i << "]: " << accessor_wrench_w[i];
    }

    msg_info() << "pinocchio gravity g = " << m_data->g;

    Eigen::VectorXd jointTorques = Eigen::VectorXd::Zero(m_model->nv);
    for (auto jointIdx = 0ul; jointIdx < m_model->njoints; ++jointIdx)
    {
      // jointForce is a spatial force so 6-vector
      const Eigen::VectorXd jointForce = sofa::rigidbodydynamics::vectorToEigen(accessor_wrench_w[jointIdx], 6);
      pinocchio::Data::Matrix6x J(6, m_model->nv);
      pinocchio::getJointJacobian(*m_model, *m_data, jointIdx, pinocchio::LOCAL, J);
      jointTorques += J.transpose() * jointForce;
    }

    // for(auto i = 0ul; i < jointTorques.size(); ++i)
    // {
    //   accessor_dgdqT_w[i] = jointTorques[i];
    // }

  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::draw(const core::visual::VisualParams *vparams)
  {
    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::setModel(const std::shared_ptr<pinocchio::Model> &model)
  {
    assert(model);
    m_model = model;
    // build model data
    m_data = std::make_shared<pinocchio::Data>(*m_model);
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::setCollisionModel(const std::shared_ptr<pinocchio::GeometryModel> &collisionModel)
  {
    assert(collisionModel);
    m_collisionModel = collisionModel;
    // build collision data
    m_collisionData = std::make_shared<pinocchio::GeometryData>(*m_collisionModel);
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::setVisualModel(const std::shared_ptr<pinocchio::GeometryModel> &visualModel)
  {
    assert(visualModel);
    m_visualModel = visualModel;
    // build visual data
    m_visualData = std::make_shared<pinocchio::GeometryData>(*m_visualModel);
  }

  template <class TIn, class TInRoot, class TOut>
  const std::shared_ptr<pinocchio::GeometryData> &KinematicChainMapping<TIn, TInRoot, TOut>::collisionData() const
  {
    return m_collisionData;
  }

  template <class TIn, class TInRoot, class TOut>
  const std::shared_ptr<pinocchio::GeometryData> &KinematicChainMapping<TIn, TInRoot, TOut>::visualData() const
  {
    return m_visualData;
  }

} // namespace sofa::component::mapping
