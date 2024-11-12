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
#include <pinocchio/algorithm/frames.hpp>

using namespace sofa::defaulttype;

namespace {
  constexpr auto kMinTorque = 1.e-6;

  constexpr auto kMinTorqueSqrd = std::pow(kMinTorque, 2);
}

namespace sofa::component::mapping::nonlinear
{
  template <class TIn, class TInRoot, class TOut>
  KinematicChainMapping<TIn, TInRoot, TOut>::KinematicChainMapping()
      : d_indexFromRoot(initData(&d_indexFromRoot, 0u, "indexInput2", "Corresponding index if the base of the articulated system is attached to input2. Default is last index."))
  {
    this->addUpdateCallback("checkIndexFromRoot", {&d_indexFromRoot}, [this](const core::DataTracker &t)
                            {
            SOFA_UNUSED(t);
            checkIndexFromRoot();
            return sofa::core::objectmodel::ComponentState::Valid; }, {&d_componentState});
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

    if (not this->getFromModels2().empty())
    {
      m_fromRootModel = this->getFromModels2()[0];
      msg_info() << "Root Model found : Name = " << m_fromRootModel->getName();
      checkIndexFromRoot();
    }

    Inherit::init();

    msg_info() << "======= kinematicChainMapping::init isMechanical = " << this->isMechanical();
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
      const core::MechanicalParams *mparams,
      const type::vector<DataVecCoord_t<Out> *> &dataVecOutPos,
      const type::vector<const DataVecCoord_t<In> *> &dataVecInPos,
      const type::vector<const DataVecCoord_t<InRoot> *> &dataVecInRootPos)
  {
    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    if (dataVecOutPos.empty() or dataVecInPos.empty())
      return;

    if (dataVecOutPos.size() != 1)
    {
      msg_error() << "KinematicChainMapping only supports output vector of size 2 (one element for joints, one element for frames)";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    for(auto i = 0; i < 2; ++i)
    {
      msg_info() << "dataVecOutPos[" << i << "] = " << dataVecOutPos[i];
    }

    if (dataVecInPos.size() > 1)
    {
      msg_error() << "KinematicChainMapping only supports input vector of size 1";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    if (dataVecInRootPos.size() > 1)
    {
      msg_error() << "KinematicChainMapping only supports input root vector of size 1";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    msg_info() << "========= KinematicChainMapping apply main";

    VecCoord_t<Out> _dataVecOutPosJoints(m_model->njoints);
    helper::ReadAccessor< DataVecCoord_t<Out> > ra_dataVecOutPos = *dataVecOutPos[0];
    for(auto i = 0ul; i < m_model->njoints; ++i)
    {
      _dataVecOutPosJoints[i] = ra_dataVecOutPos[i];
    }
    VecCoord_t<Out> _dataVecOutPosFrames(m_extraFrames.size());
    for(auto i = 0ul; i < m_extraFrames.size(); ++i)
    {
      _dataVecOutPosFrames[i] = ra_dataVecOutPos[i + m_model->njoints];
    }

    if (not dataVecInRootPos.empty())
    {
      apply(mparams, _dataVecOutPosJoints, _dataVecOutPosFrames, *dataVecInPos[0], dataVecInRootPos[0]);
    }
    else
    {
      apply(mparams, _dataVecOutPosJoints, _dataVecOutPosFrames, *dataVecInPos[0], nullptr);
    }

    helper::WriteAccessor< DataVecCoord_t<Out> > w_dataVecOutPos = *dataVecOutPos[0];
    for(auto i = 0ul; i < m_model->njoints; ++i)
    {
      w_dataVecOutPos[i] = _dataVecOutPosJoints[i];
    }
    for(auto i = 0ul; i < m_extraFrames.size(); ++i)
    {
      w_dataVecOutPos[i + m_model->njoints] = _dataVecOutPosFrames[i];
    }
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJ(
      const core::MechanicalParams *mparams, const type::vector<DataVecDeriv_t<Out> *> &dataVecOutVel,
      const type::vector<const DataVecDeriv_t<In> *> &dataVecInVel,
      const type::vector<const DataVecDeriv_t<InRoot> *> &dataVecInRootVel)
  {
    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    if (dataVecOutVel.empty() or dataVecInVel.empty())
      return;

    if (dataVecOutVel.size() != 1)
    {
      msg_error() << "KinematicChainMapping only supports output vector of size 2 (one element for joints, one element for frames)";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    for(auto i = 0; i < 2; ++i)
    {
      msg_info() << "dataVecOutVel[" << i << "] = " << dataVecOutVel[i];
    }

    if (dataVecInVel.size() > 1)
    {
      msg_error() << "KinematicChainMapping only supports input vector of size 1";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    if (dataVecInRootVel.size() > 1)
    {
      msg_error() << "KinematicChainMapping only supports input root vector of size 1";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    msg_info() << "========= KinematicChainMapping applyJ main";

    VecDeriv_t<Out> _dataVecOutVelJoints(m_model->njoints);
    helper::ReadAccessor< DataVecDeriv_t<Out> > ra_dataVecOutVel = *dataVecOutVel[0];
    for(auto i = 0ul; i < m_model->njoints; ++i)
    {
      _dataVecOutVelJoints[i] = ra_dataVecOutVel[i];
    }
    VecDeriv_t<Out> _dataVecOutVelFrames(m_extraFrames.size());
    for(auto i = 0ul; i < m_extraFrames.size(); ++i)
    {
      _dataVecOutVelFrames[i] = ra_dataVecOutVel[i + m_model->njoints];
    }

    if (not dataVecInRootVel.empty())
    {
      applyJ(mparams, _dataVecOutVelJoints, _dataVecOutVelFrames, *dataVecInVel[0], dataVecInRootVel[0]);
    }
    else
    {
      applyJ(mparams, _dataVecOutVelJoints, _dataVecOutVelFrames, *dataVecInVel[0], nullptr);
    }

    helper::WriteAccessor< DataVecDeriv_t<Out> > wa_dataVecVelPos = *dataVecOutVel[0];
    for(auto i = 0ul; i < m_model->njoints; ++i)
    {
      wa_dataVecVelPos[i] = _dataVecOutVelJoints[i];
    }
    for(auto i = 0ul; i < m_extraFrames.size(); ++i)
    {
      wa_dataVecVelPos[i + m_model->njoints] = _dataVecOutVelFrames[i];
    }
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJT(
      const core::MechanicalParams *mparams, const type::vector<DataVecDeriv_t<In> *> &dataVecOut1Force,
      const type::vector<DataVecDeriv_t<InRoot> *> &dataVecOut2Force,
      const type::vector<const DataVecDeriv_t<Out> *> &dataVecInForce)
  {
    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    if (dataVecInForce.empty() or dataVecOut1Force.empty())
      return;

    if (dataVecInForce.size() != 1)
    {
      msg_error() << "KinematicChainMapping only supports output vector of size 2 (one element for joints, one element for frames)";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    if (dataVecOut1Force.size() > 1)
    {
      msg_error() << "KinematicChainMapping only supports input vector of size 1";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    if (dataVecOut2Force.size() > 1)
    {
      msg_error() << "KinematicChainMapping only supports input root vector of size 1";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    msg_info() << "========= KinematicChainMapping applyJT main";
    
    VecDeriv_t<Out> _dataVecInForceJoints(m_model->njoints);
    helper::ReadAccessor< DataVecDeriv_t<Out> > ra_dataVecInForce = *dataVecInForce[0];
    for(auto i = 0ul; i < m_model->njoints; ++i)
    {
      _dataVecInForceJoints[i] = ra_dataVecInForce[i];
    }

    if (not dataVecOut2Force.empty())
    {
      applyJT(mparams, *dataVecOut1Force[0], dataVecOut2Force[0], _dataVecInForceJoints);
    }
    else
    {
      applyJT(mparams, *dataVecOut1Force[0], nullptr, _dataVecInForceJoints);
    }

    // helper::WriteAccessor< DataVecDeriv_t<Out> > wa_dataVecInForce = *dataVecInForce[0];
    // for(auto i = 0ul; i < m_model->njoints; ++i)
    // {
    //   wa_dataVecInForce[i] = _dataVecInForceJoints[i];
    // }
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJT(
      const core::ConstraintParams *cparams, const type::vector<DataMatrixDeriv_t<In> *> &dataMatOut1Const,
      const type::vector<DataMatrixDeriv_t<InRoot> *> &dataMatOut2Const,
      const type::vector<const DataMatrixDeriv_t<Out> *> &dataMatInConst)
  {
    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    if (dataMatInConst.empty() or dataMatOut1Const.empty())
      return;

    if (dataMatInConst.size() != 1)
    {
      msg_error() << "KinematicChainMapping only supports output vector of size 2 (one element for joints, one element for frames)";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    if (dataMatOut1Const.size() > 1)
    {
      msg_error() << "KinematicChainMapping only supports input vector of size 1";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    if (dataMatOut2Const.size() > 1)
    {
      msg_error() << "KinematicChainMapping only supports input root vector of size 1";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    msg_info() << "========= KinematicChainMapping applyJT matrix main";
    if (not dataMatOut2Const.empty())
    {
      applyJT(cparams, *dataMatOut1Const[0], dataMatOut2Const[0], *dataMatInConst[0]);
    }
    else
    {
      applyJT(cparams, *dataMatOut1Const[0], nullptr, *dataMatInConst[0]);
    }
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
  void KinematicChainMapping<TIn, TInRoot, TOut>::setBodyCoMFrames(const std::vector<pinocchio::FrameIndex> &bodyCoMFrames)
  {
    m_bodyCoMFrames = bodyCoMFrames;
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::checkIndexFromRoot()
  {
    sofa::Size rootSize = m_fromRootModel->getSize();
    if (d_indexFromRoot.isSet())
    {
      if (d_indexFromRoot.getValue() >= rootSize)
      {
        msg_warning() << d_indexFromRoot.getName() << ", " << d_indexFromRoot.getValue() << ", is larger than input2's size, " << rootSize
                      << ". Using the default value instead which in this case will be " << rootSize - 1;
        d_indexFromRoot.setValue(rootSize - 1);
      }
    }
    else
    {
      d_indexFromRoot.setValue(rootSize - 1); // default is last index
    }
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::apply(
      const core::MechanicalParams *mparams,
      VecCoord_t<Out> &_out_joints,
      VecCoord_t<Out> &_out_frames,
      const DataVecCoord_t<In> &in,
      const DataVecCoord_t<InRoot> *inRoot)
  {
    SOFA_UNUSED(mparams);

    assert(m_model);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    msg_info() << "========= KinematicChainMapping apply entering...";

    helper::ReadAccessor< DataVecCoord_t<In> > _in = in;
    // helper::WriteOnlyAccessor< DataVecCoord_t<Out> > _out_joints = out_joints;
    // helper::WriteOnlyAccessor< DataVecCoord_t<Out> > _out_frames = out_frames;

    msg_info() << "========= KinematicChainMapping  apply _out_joints.size = " << _out_joints.size() << " (should be " <<  m_model->njoints << ")";
    msg_info() << "========= KinematicChainMapping apply _out_frames.size = " << _out_frames.size() << " (should be " <<  m_extraFrames.size() << ")";
    msg_info() << "======= kinematicChainMapping apply isMechanical = " << this->isMechanical();


    Eigen::VectorXd q = Eigen::VectorXd::Zero(m_model->nq);
    if (m_fromRootModel and inRoot != nullptr)
    {
      helper::ReadAccessor< DataVecCoord_t<InRoot> > _inRoot = inRoot;

      const sofa::defaulttype::RigidCoord<3, double> &rootPose_w = _inRoot[d_indexFromRoot.getValue()];
      q.head<7>() = sofa::rigidbodydynamics::se3ToEigen(rootPose_w);
      q.tail(_in.size()) = sofa::rigidbodydynamics::vectorVec1ToEigen(_in, m_model->nq);
    }
    else
    {
      q = sofa::rigidbodydynamics::vectorVec1ToEigen(_in, m_model->nq);
    }

    // Computes joints Jacobians and forward kinematics. Jacobians will
    // be used by applyJ and not apply function, but this is done here
    // to avoid duplicate computation of forward kinematics

    // Note that as mentionned in pinocchio documentation, calling computeJointJacobians(model,data,q), 
    // then updateFramePlacements(model,data) and then call getJointJacobian(model,data,jointId,rf,J),
    // as done here, is equivalent to call computeFrameJacobian(model,data,jointId,rf,J), but here
    // forwardKinematics and updateFramePlacements are fully computed.
    pinocchio::computeJointJacobians(*m_model, *m_data, q);
    // msg_info() << " fwd kinematics & joint jacobians computed";
    pinocchio::updateFramePlacements(*m_model, *m_data);
    // msg_info() << " frames placements updated";


    // Single output vector of size njoints
    for (auto jointIdx = 0ul; jointIdx < m_model->njoints; ++jointIdx)
    {
      const auto &frameIdx = m_bodyCoMFrames[jointIdx];
      _out_joints[jointIdx] = sofa::rigidbodydynamics::se3ToSofaType(m_data->oMf[frameIdx]);
    }
    // Single other output vector of size num extra frames
    // assert(_out_frames.size() == m_extraFrames.size());

    // for (auto i = 0ul; i < m_extraFrames.size(); ++i)
    // {
    //   const auto &frameIdx = m_extraFrames[i];
    //   _out_frames[i] = sofa::rigidbodydynamics::se3ToSofaType(m_data->oMf[frameIdx]);
    // }
    msg_info() << "========= KinematicChainMapping apply exiting...";
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJ(
      const core::MechanicalParams* mparams, 
      // DataVecDeriv_t<Out>& out_joints,
      // DataVecDeriv_t<Out>& out_frames,
      VecDeriv_t<Out>& _out_joints,
      VecDeriv_t<Out>& _out_frames,
      const DataVecDeriv_t<In>& in,
      const DataVecDeriv_t<InRoot>* inRoot)
  {
    SOFA_UNUSED(mparams);

    assert(m_model);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    msg_info() << "========= KinematicChainMapping applyJ entering...";
    helper::ReadAccessor< DataVecDeriv_t<In> > _in = in;
    // helper::WriteOnlyAccessor< DataVecDeriv_t<Out> > _out_joints = out_joints;
    // helper::WriteOnlyAccessor< DataVecDeriv_t<Out> > _out_frames = out_frames;

    msg_info() << "========= KinematicChainMapping  applyJ _out_joints.size = " << _out_joints.size() << " (should be " <<  m_model->njoints << ")";
    msg_info() << "========= KinematicChainMapping applyJ _out_frames.size = " << _out_frames.size() << " (should be " <<  m_extraFrames.size() << ")";
    msg_info() << "======= kinematicChainMapping applyJ isMechanical = " << this->isMechanical();

    // map in configuration to pinocchio
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(m_model->nv);
    if (m_fromRootModel and inRoot != nullptr)
    {
      helper::ReadAccessor< DataVecDeriv_t<InRoot> > _inRoot = inRoot;

      const sofa::defaulttype::RigidDeriv<3, double> &rootVel_w = _inRoot[d_indexFromRoot.getValue()];
      dq.head<6>() = sofa::rigidbodydynamics::spatialVelocityToEigen(rootVel_w);
      dq.tail(_in.size()) = sofa::rigidbodydynamics::vectorVec1ToEigen(_in, m_model->nv);
    }
    else
    {
      dq = sofa::rigidbodydynamics::vectorVec1ToEigen(_in, m_model->nv);
    }

    // Single output vector of size njoints
    assert(_out_joints.size() == m_model->njoints);

    
    for (auto jointIdx = 0ul; jointIdx < m_model->njoints; ++jointIdx)
    {
      pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, m_model->nv);
      const auto &frameIdx = m_bodyCoMFrames[jointIdx];
      pinocchio::getFrameJacobian(*m_model, *m_data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, J);
      Eigen::VectorXd dg = J * dq;
      _out_joints[jointIdx] = sofa::rigidbodydynamics::vec6ToSofaType<Eigen::VectorXd>(dg);
    }

    // assert(_out_frames.size() == m_extraFrames.size());


    // // Single output vector of size number of extra frames
    // for (auto i = 0ul; i < m_extraFrames.size(); ++i)
    // {
    //   pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, m_model->nv);
    //   const auto &frameIdx = m_extraFrames[i];
    //   pinocchio::getFrameJacobian(*m_model, *m_data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, J);
    //   Eigen::VectorXd dg = J * dq;
    //   _out_frames[i] = sofa::rigidbodydynamics::vec6ToSofaType<Eigen::VectorXd>(dg);
    // }

    msg_info() << "========= KinematicChainMapping applyJ exiting...";
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJT(
      const core::MechanicalParams *mparams, 
      DataVecDeriv_t<In>& out,
      DataVecDeriv_t<InRoot>* outRoot,
      const VecDeriv_t<Out>& _in_joints)
      // const DataVecDeriv_t<Out>& in_joints)
  {
    SOFA_UNUSED(mparams);

    assert(m_model);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    msg_info() << "========= KinematicChainMapping applyJT entering...";
    // maps spatial forces applied on each body to torques on joints

    // out will be the resulting torque on each joint
    // size = nv if no root joint, nv+6 if using free-flyer root joint
    // XXX ERROR: nv-6 if using free-flyer root joint -> confirm
    helper::WriteAccessor<DataVecDeriv_t<In>> _out(out);

    msg_info() << "=================== out.size() = " << _out.size();
    msg_info() << "=================== nv = " << m_model->nv;
    // in is a vector of spatial forces applied at each joint (actually frame)
    // size = njoints
    // helper::ReadAccessor<DataVecDeriv_t<Out>> _in_joints(in_joints);

    Eigen::VectorXd jointsTorques = Eigen::VectorXd::Zero(m_model->nv);
    for (auto jointIdx = 0ul; jointIdx < m_model->njoints; ++jointIdx)
    {
      // jointForce is a spatial force so 6-vector
      const Eigen::VectorXd jointForce = sofa::rigidbodydynamics::vectorToEigen(_in_joints[jointIdx], 6);
      pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, m_model->nv);
      const auto &frameIdx = m_bodyCoMFrames[jointIdx];

      pinocchio::getFrameJacobian(*m_model, *m_data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, J);
      jointsTorques += J.transpose() * jointForce;
    }

    if (m_fromRootModel and outRoot != nullptr)
    {
      // joint torques contains first 6 parameters for the root joint, and nv-6 parameters for other joints
      helper::WriteAccessor<DataVecDeriv_t<InRoot>> _outRoot(outRoot);
      // write (add) root joint spatial force
      _outRoot[0] += sofa::rigidbodydynamics::vec6ToSofaType(jointsTorques.head<6>());
      // write (add) joint torques
      for (auto i = 0ul; i < jointsTorques.size() - 6; ++i)
      {
        _out[i][0] += jointsTorques[i + 6];
      }
    }
    else
    {
      // no root joints case, only write (add) joint torques
      for (auto i = 0ul; i < jointsTorques.size(); ++i)
      {
        _out[i][0] += jointsTorques[i];
      }
    }
    msg_info() << "========= KinematicChainMapping applyJT exiting...";
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJT(
      const core::ConstraintParams *cparams, 
      DataMatrixDeriv_t<In>& out,
      DataMatrixDeriv_t<InRoot>* outRoot,
      const DataMatrixDeriv_t<Out>& in_joints)
  {
    SOFA_UNUSED(cparams);

    assert(m_model);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    msg_info() << "========= KinematicChainMapping applyJT matrix entering...";
    // out will be the resulting torque on each joint
    helper::WriteAccessor<DataMatrixDeriv_t<In>> _out(out);
    helper::ReadAccessor<DataMatrixDeriv_t<Out>> _in_joints(in_joints);

    // row = constraint
    for (auto rowIt = _in_joints->begin(); rowIt != _in_joints->end(); ++rowIt)
    {
      Eigen::VectorXd jointsTorques = Eigen::VectorXd::Zero(m_model->nv);
      // col = dof (bodies)
      for (auto colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
      {
        // retrieve body frame index (centered at CoM) for each body
        const auto jointIdx = colIt.index();
        const auto &frameIdx = m_bodyCoMFrames[jointIdx];

        // get jacobian associated to this body
        pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, m_model->nv);
        pinocchio::getFrameJacobian(*m_model, *m_data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, J);

        // sum joint torques for all bodies
        jointsTorques += J.transpose() * sofa::rigidbodydynamics::spatialVelocityToEigen(colIt.val());
      }

      if (m_fromRootModel and outRoot != nullptr)
      {
        if(jointsTorques.squaredNorm() > kMinTorqueSqrd)// dismiss dofs resulting in null torque
        {
          // write (add) root joint spatial force
          helper::WriteAccessor<DataMatrixDeriv_t<InRoot>> _outRoot(outRoot);

          auto oRoot = _outRoot->writeLine(rowIt.index());
          const Deriv_t<InRoot> rootWrench(sofa::rigidbodydynamics::vec6ToSofaType(jointsTorques.head<6>()));
          oRoot.addCol(0, rootWrench);
          // write summed joint torques for this constraint to output
          auto outRowIt = _out->writeLine(rowIt.index());
          for (auto i = 0ul; i < jointsTorques.size() - 6; ++i)
          {
            const Deriv_t<In> torque(jointsTorques[i + 6]); // InDeriv is Vec1 type
            outRowIt.addCol(i, torque);
          }
        }
      }
      else
      {
        // write summed joint torques for this constraint to output
        if(jointsTorques.squaredNorm() > kMinTorqueSqrd) // dismiss dofs resulting in null torque
        {
          auto outRowIt = _out->writeLine(rowIt.index());
          for (auto i = 0ul; i < jointsTorques.size(); ++i)
          {
            const Deriv_t<In> torque(jointsTorques[i]); // InDeriv is Vec1 type
            outRowIt.addCol(i, torque);
          }
        }
      }
    }

    // // print output matrices
    // if (m_fromRootModel and outRoot != nullptr)
    // {
    //   msg_info() << "outRootWrench CRS matrix:";
    //     helper::WriteAccessor<DataMatrixDeriv_t<InRoot>> _outRoot(outRoot);
    //   for (auto rowIt = _outRoot->begin(); rowIt != _outRoot->end(); ++rowIt)
    //   {
    //     for (auto colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
    //     {
    //       msg_info() << "row[" << rowIt.index() << "], col[" << colIt.index() << "]:" << colIt.val();
    //     }
    //   }
    // }
    // msg_info() << "outTorques CRS matrix:";
    // for (auto rowIt = _out->begin(); rowIt != _out->end(); ++rowIt)
    // {
    //   for (auto colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
    //   {
    //     msg_info() << "row[" << rowIt.index() << "], col[" << colIt.index() << "]:" << colIt.val();
    //   }
    // }
    msg_info() << "========= KinematicChainMapping applyJT matrix exiting...";
  }

} // namespace sofa::component::mapping::nonlinear
