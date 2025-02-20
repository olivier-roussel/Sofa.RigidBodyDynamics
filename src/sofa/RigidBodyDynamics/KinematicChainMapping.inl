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

#include <sofa/RigidBodyDynamics/Types.h>

#include <sofa/RigidBodyDynamics/Conversions.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/visual/DrawTool.h>

#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/frames.hpp>

using namespace sofa::defaulttype;
using namespace sofa::rigidbodydynamics;

namespace {
  constexpr auto kMinTorque = 1.e-6;

  constexpr auto kMinTorqueSqrd = kMinTorque*kMinTorque;
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
    // msg_info() << "========= KinematicChainMapping init";
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
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::bwdInit()
  {
    // msg_info() << "========= KinematicChainMapping bwdInit";
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::reset()
  {
    // msg_info() << "========= KinematicChainMapping reset";

    init();
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::apply(
      const core::MechanicalParams *mparams,
      const type::vector<DataVecCoord_t<Out> *> &dataVecOutPos,
      const type::vector<const DataVecCoord_t<In> *> &dataVecInPos,
      const type::vector<const DataVecCoord_t<InRoot> *> &dataVecInRootPos)
  {
    SOFA_UNUSED(mparams);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    if (dataVecOutPos.empty() or dataVecInPos.empty())
      return;

    if (dataVecOutPos.size() > 1)
    {
      msg_error() << "KinematicChainMapping only supports output vector of size 1";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
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


    // msg_info() << "========= KinematicChainMapping apply main";
    if (not dataVecInRootPos.empty())
    {
      apply(dataVecOutPos[0]->beginEdit(), &dataVecInPos[0]->getValue(), &dataVecInRootPos[0]->getValue());
    }
    else
    {
      apply(dataVecOutPos[0]->beginEdit(), &dataVecInPos[0]->getValue(), nullptr);
    }
    dataVecOutPos[0]->endEdit();
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJ(
      const core::MechanicalParams *mparams, const type::vector<DataVecDeriv_t<Out> *> &dataVecOutVel,
      const type::vector<const DataVecDeriv_t<In> *> &dataVecInVel,
      const type::vector<const DataVecDeriv_t<InRoot> *> &dataVecInRootVel)
  {
    SOFA_UNUSED(mparams);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    if (dataVecOutVel.empty() or dataVecInVel.empty())
      return;

    if (dataVecOutVel.size() > 1)
    {
      msg_error() << "KinematicChainMapping only supports output vector of size 1";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
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

    // msg_info() << "========= KinematicChainMapping applyJ main";
    if (not dataVecInRootVel.empty())
    {
      applyJ(dataVecOutVel[0]->beginEdit(), &dataVecInVel[0]->getValue(), &dataVecInRootVel[0]->getValue());
    }
    else
    {
      applyJ(dataVecOutVel[0]->beginEdit(), &dataVecInVel[0]->getValue(), nullptr);
    }
    dataVecOutVel[0]->endEdit();
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJT(
      const core::MechanicalParams *mparams, const type::vector<DataVecDeriv_t<In> *> &dataVecOut1Force,
      const type::vector<DataVecDeriv_t<InRoot> *> &dataVecOut2Force,
      const type::vector<const DataVecDeriv_t<Out> *> &dataVecInForce)
  {
    SOFA_UNUSED(mparams);

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

    // msg_info() << "========= KinematicChainMapping applyJT main";
    if (not dataVecOut2Force.empty())
    {
      applyJT(dataVecOut1Force[0]->beginEdit(), dataVecOut2Force[0]->beginEdit(), &dataVecInForce[0]->getValue());
      dataVecOut2Force[0]->endEdit();
    }
    else
    {
      applyJT(dataVecOut1Force[0]->beginEdit(), nullptr, &dataVecInForce[0]->getValue());
    }
    dataVecOut1Force[0]->endEdit();
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJT(
      const core::ConstraintParams *cparams, const type::vector<DataMatrixDeriv_t<In> *> &dataMatOut1Const,
      const type::vector<DataMatrixDeriv_t<InRoot> *> &dataMatOut2Const,
      const type::vector<const DataMatrixDeriv_t<Out> *> &dataMatInConst)
  {
    SOFA_UNUSED(cparams);

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

    // msg_info() << "========= KinematicChainMapping applyJT matrix main";
    if (not dataMatOut2Const.empty())
    {
      applyJT(dataMatOut1Const[0]->beginEdit(), dataMatOut2Const[0]->beginEdit(), &dataMatInConst[0]->getValue());
      dataMatOut2Const[0]->endEdit();
    }
    else
    {
      applyJT(dataMatOut1Const[0]->beginEdit(), nullptr, &dataMatInConst[0]->getValue());
    }
    dataMatOut1Const[0]->endEdit();
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
      VecCoord_t<Out>* out,
      const VecCoord_t<In>* in,
      const VecCoord_t<InRoot>* inRoot)
  {
    assert(m_model);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    const int num_joints = m_model->njoints - kSkipUniverse; // if we do not want universe joint

    // msg_info() << "========= KinematicChainMapping apply entering...";

    Eigen::VectorXd q = Eigen::VectorXd::Zero(m_model->nq);
    if (m_fromRootModel and inRoot != nullptr)
    {
      const sofa::defaulttype::RigidCoord<3, double> &rootPose_w = (*inRoot)[d_indexFromRoot.getValue()];
      q.head<7>() = se3ToEigen(rootPose_w);
      q.tail((*in).size()) = vectorVec1ToEigen(*in, m_model->nq);
    }
    else
    {
      q = vectorVec1ToEigen(*in, m_model->nq);
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

    // Single output vector of size njoints + number of extra frames
    assert((*out).size() == (num_joints + m_extraFrames.size()));

    // Write joints
    for (JointIndex jointIdx = 0ul; jointIdx < num_joints; ++jointIdx)
    {
      const auto &frameIdx = m_bodyCoMFrames[jointIdx];
      (*out)[jointIdx] = se3ToSofaType(m_data->oMf[frameIdx]);
    }
    
    // Write frames
    for (auto i = 0ul; i < m_extraFrames.size(); ++i)
    {
      const auto &frameIdx = m_extraFrames[i];
      // msg_info() << "out.size = " << (*out).size() << " / index = " << i + m_model->njoints - 1 << " / i = " << i;
      // msg_info() << "m_data->oMf.size = " << m_data->oMf.size() << " / frameIdx = " << frameIdx;
      (*out)[i + num_joints] = se3ToSofaType(m_data->oMf[frameIdx]);
    }
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJ(
      VecDeriv_t<Out>* out,
      const VecDeriv_t<In>* in,
      const VecDeriv_t<InRoot>* inRoot)
  {
    assert(m_model);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    const int num_joints = m_model->njoints - kSkipUniverse; // if we do not want universe joint

    // msg_info() << "========= KinematicChainMapping applyJ entering...";

    // map in configuration to pinocchio
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(m_model->nv);
    if (m_fromRootModel and inRoot != nullptr)
    {
      const sofa::defaulttype::RigidDeriv<3, double> &rootVel_w = (*inRoot)[d_indexFromRoot.getValue()];
      dq.head<6>() = spatialVelocityToEigen(rootVel_w);
      dq.tail((*in).size()) = vectorVec1ToEigen(*in, m_model->nv);
    }
    else
    {
      dq = vectorVec1ToEigen(*in, m_model->nv);
    }

    // Single output vector of size njoints + number of extra frames
    assert((*out).size() == (num_joints + m_extraFrames.size()));

    // Write joints
    for (JointIndex jointIdx = 0ul; jointIdx < num_joints; ++jointIdx)
    {
      pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, m_model->nv);
      const auto &frameIdx = m_bodyCoMFrames[jointIdx];
      pinocchio::getFrameJacobian(*m_model, *m_data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, J);
      Eigen::VectorXd dg = J * dq;
      (*out)[jointIdx] = vec6ToSofaType<Eigen::VectorXd>(dg);
    }

    // Write frames
    for (auto i = 0ul; i < m_extraFrames.size(); ++i)
    {
      pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, m_model->nv);
      const auto &frameIdx = m_extraFrames[i];
      pinocchio::getFrameJacobian(*m_model, *m_data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, J);
      Eigen::VectorXd dg = J * dq;
      (*out)[i + num_joints] = vec6ToSofaType<Eigen::VectorXd>(dg);
    }
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJT(
     VecDeriv_t<In>* out,
     VecDeriv_t<InRoot>* outRoot,
     const VecDeriv_t<Out>* in)
  {
    assert(m_model);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    const int num_joints = m_model->njoints - kSkipUniverse; // if we do not want universe joint
  
    // maps spatial forces applied on each body to torques on joints
    // msg_info() << "========= KinematicChainMapping applyJT entering...";

    // out will be the resulting torque on each joint
    // size = nv if no root joint, nv-6 if using free-flyer root joint

    // in is a vector of spatial forces applied at each joint (actually frame)
    // size = njoints

    Eigen::VectorXd jointsTorques = Eigen::VectorXd::Zero(m_model->nv);
    for (JointIndex jointIdx = 0ul; jointIdx < num_joints; ++jointIdx)
    {
      // jointForce is a spatial force so 6-vector
      const Eigen::VectorXd jointForce = vectorToEigen((*in)[jointIdx], 6);
      pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, m_model->nv);
      const auto &frameIdx = m_bodyCoMFrames[jointIdx];

      pinocchio::getFrameJacobian(*m_model, *m_data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, J);
      jointsTorques += J.transpose() * jointForce;
    }
    for (auto i = 0ul; i < m_extraFrames.size(); ++i)
    {
      // frameForce is a spatial force so 6-vector
      const Eigen::VectorXd frameForce = vectorToEigen((*in)[num_joints + i], 6);
      pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, m_model->nv);
      const auto &frameIdx = m_extraFrames[i];

      pinocchio::getFrameJacobian(*m_model, *m_data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, J);
      jointsTorques += J.transpose() * frameForce;
    }

    if (m_fromRootModel and outRoot != nullptr)
    {
      // joint torques contains first 6 parameters for the root joint, and nv-6 parameters for other joints
      // write (add) root joint spatial force
      (*outRoot)[0] += vec6ToSofaType(jointsTorques.head<6>());
      // write (add) joint torques
      for (auto i = 0ul; i < jointsTorques.size() - 6; ++i)
      {
        (*out)[i][0] += jointsTorques[i + 6];
      }
    }
    else
    {
      // no root joints case, only write (add) joint torques
      for (auto i = 0ul; i < jointsTorques.size(); ++i)
      {
        (*out)[i][0] += jointsTorques[i];
      }
    }
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJT(
      MatrixDeriv_t<In>* out,
      MatrixDeriv_t<InRoot>* outRoot,
      const MatrixDeriv_t<Out>* in)
  {
    assert(m_model);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    const int num_joints = m_model->njoints - kSkipUniverse; // if we do not want universe joint
   
    // msg_info() << "========= KinematicChainMapping applyJT matrix entering...";
    // out will be the resulting torque on each joint
    // out->clear();
    // if(outRoot)
    // {
    //   outRoot->clear();
    // }
    // print output matrices
    if (m_fromRootModel and outRoot != nullptr)
    {
      msg_info() << "BEGIN outRootWrench CRS matrix:";
        // helper::WriteAccessor<DataMatrixDeriv_t<InRoot>> _outRoot(outRoot);
      for (auto rowIt = outRoot->begin(); rowIt != outRoot->end(); ++rowIt)
      {
        for (auto colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
        {
          msg_info() << "row[" << rowIt.index() << "], col[" << colIt.index() << "]:" << colIt.val();
        }
      }
    }
    msg_info() << "BEGIN outTorques CRS matrix:";
    for (auto rowIt = out->begin(); rowIt != out->end(); ++rowIt)
    {
      for (auto colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
      {
        msg_info() << "row[" << rowIt.index() << "], col[" << colIt.index() << "]:" << colIt.val();
      }
    }

    // row = constraint
    for (auto rowIt = in->begin(); rowIt != in->end(); ++rowIt)
    {
      Eigen::VectorXd jointsTorques = Eigen::VectorXd::Zero(m_model->nv);
      // col = dof (bodies)
      for (auto colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
      {
        // retrieve body frame index (centered at CoM) for each body
        pinocchio::FrameIndex frameIdx;
        const JointIndex jointIdx = colIt.index();
        if(jointIdx < num_joints)
        {
          frameIdx = m_bodyCoMFrames[jointIdx];
        }
        else
        {
          frameIdx = m_extraFrames[jointIdx - num_joints];
        }
        // msg_info() << "==== applyJT jointIdx = " << jointIdx << " / frameIdx = " << frameIdx;

        // get jacobian associated to this body
        pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, m_model->nv);
        pinocchio::getFrameJacobian(*m_model, *m_data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, J);

        // sum joint torques for all bodies
        jointsTorques += J.transpose() * spatialVelocityToEigen(colIt.val());
      }

      if (m_fromRootModel and outRoot != nullptr)
      {
        if(jointsTorques.squaredNorm() > kMinTorqueSqrd)// dismiss dofs resulting in null torque
        {
          // write (add) root joint spatial force
          auto oRoot = outRoot->writeLine(rowIt.index());
          const Deriv_t<InRoot> rootWrench(vec6ToSofaType(jointsTorques.head<6>()));
          oRoot.addCol(0, rootWrench);
          // write summed joint torques for this constraint to output
          auto outRowIt = out->writeLine(rowIt.index());
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
          auto outRowIt = out->writeLine(rowIt.index());
          for (auto i = 0ul; i < jointsTorques.size(); ++i)
          {
            const Deriv_t<In> torque(jointsTorques[i]); // InDeriv is Vec1 type
            // msg_info() << "writing row[" << rowIt.index() << "], col[" << i << "]:" << torque;
            outRowIt.addCol(i, torque);
          }
        }
      }
    }

    // print output matrices
    if (m_fromRootModel and outRoot != nullptr)
    {
      msg_info() << "END outRootWrench CRS matrix:";
        // helper::WriteAccessor<DataMatrixDeriv_t<InRoot>> _outRoot(outRoot);
      for (auto rowIt = outRoot->begin(); rowIt != outRoot->end(); ++rowIt)
      {
        for (auto colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
        {
          msg_info() << "row[" << rowIt.index() << "], col[" << colIt.index() << "]:" << colIt.val();
        }
      }
    }
    msg_info() << "END outTorques CRS matrix:";
    for (auto rowIt = out->begin(); rowIt != out->end(); ++rowIt)
    {
      for (auto colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
      {
        msg_info() << "row[" << rowIt.index() << "], col[" << colIt.index() << "]:" << colIt.val();
      }
    }
  }

} // namespace sofa::component::mapping::nonlinear
