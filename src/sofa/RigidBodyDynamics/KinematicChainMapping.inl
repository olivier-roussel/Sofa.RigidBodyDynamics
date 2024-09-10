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

    if (not dataVecInRootPos.empty())
    {
      apply(mparams, *dataVecOutPos[0], *dataVecInPos[0], dataVecInRootPos[0]);
    }
    else
    {
      apply(mparams, *dataVecOutPos[0], *dataVecInPos[0], nullptr);
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

    if (not dataVecInRootVel.empty())
    {
      applyJ(mparams, *dataVecOutVel[0], *dataVecInVel[0], dataVecInRootVel[0]);
    }
    else
    {
      applyJ(mparams, *dataVecOutVel[0], *dataVecInVel[0], nullptr);
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

    if (dataVecInForce.size() > 1)
    {
      msg_error() << "KinematicChainMapping only supports output vector of size 1";
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

    if (not dataVecOut2Force.empty())
    {
      applyJT(mparams, *dataVecOut1Force[0], dataVecOut2Force[0], *dataVecInForce[0]);
    }
    else
    {
      applyJT(mparams, *dataVecOut1Force[0], nullptr, *dataVecInForce[0]);
    }
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

    if (dataMatInConst.size() > 1)
    {
      msg_error() << "KinematicChainMapping only supports output vector of size 1";
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
    assert(bodyCoMFrames);
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
      DataVecCoord_t<Out> &out,
      const DataVecCoord_t<In> &in,
      const DataVecCoord_t<InRoot> *inRoot)
  {
    SOFA_UNUSED(mparams);

    assert(m_model);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    VecCoord_t<In> inDofs = in.getValue();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(m_model->nq);
    if (m_fromRootModel and inRoot != nullptr)
    {
      VecCoord_t<InRoot> inRootVec_w = inRoot->getValue();
      // msg_info() << "inRootVec_w.size() = " << inRootVec_w.size();

      const sofa::defaulttype::RigidCoord<3, double> &rootPose_w = inRootVec_w[d_indexFromRoot.getValue()];
      q.head<7>() = sofa::rigidbodydynamics::se3ToEigen(rootPose_w);
      q.tail(inDofs.size()) = sofa::rigidbodydynamics::vectorVec1ToEigen(inDofs, m_model->nq);
    }
    else
    {
      q = sofa::rigidbodydynamics::vectorVec1ToEigen(inDofs, m_model->nq);
    }

    // Computes joints Jacobians and forward kinematics. Jacobians will
    // be used by applyJ and not apply function, but this is done here
    // to avoid duplicate computation of forward kinematics
    pinocchio::computeJointJacobians(*m_model, *m_data, q);
    // msg_info() << " fwd kinematics & joint jacobians computed";

    pinocchio::updateFramePlacements(*m_model, *m_data);
    // msg_info() << " frames placements updated";

    // Single output vector of size njoints
    // DataVecCoord_t<Out> *g_w = dataVecOutPos[0];

    helper::WriteAccessor<DataVecCoord_t<Out>> accessor_g_w(out);
    for (auto bodyIdx = 0ul; bodyIdx < m_model->nbodies; ++bodyIdx)
    {
      const auto &frameIdx = m_bodyCoMFrames[bodyIdx];
      accessor_g_w[bodyIdx] = sofa::rigidbodydynamics::se3ToSofaType(m_data->oMf[frameIdx]);
    }
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJ(
      const core::MechanicalParams* mparams, 
      DataVecDeriv_t<Out>& out,
      const DataVecDeriv_t<In>& in,
      const DataVecDeriv_t<InRoot>* inRoot)
  {
    SOFA_UNUSED(mparams);

    assert(m_model);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    // map in configuration to pinocchio
    VecDeriv_t<In> inDofs = in.getValue();

    Eigen::VectorXd dq = Eigen::VectorXd::Zero(m_model->nv);
    if (m_fromRootModel and inRoot != nullptr)
    {
      VecDeriv_t<InRoot> inRootVelVec_w = inRoot->getValue();

      const sofa::defaulttype::RigidDeriv<3, double> &rootVel_w = inRootVelVec_w[d_indexFromRoot.getValue()];
      dq.head<6>() = sofa::rigidbodydynamics::spatialVelocityToEigen(rootVel_w);
      dq.tail(inDofs.size()) = sofa::rigidbodydynamics::vectorVec1ToEigen(inDofs, m_model->nv);
    }
    else
    {
      dq = sofa::rigidbodydynamics::vectorVec1ToEigen(inDofs, m_model->nv);
    }

    // Single output vector of size njoints
    assert(out.getValue().size() == m_model->njoints);

    helper::WriteAccessor<DataVecDeriv_t<Out>> accessor_dgdq_w(out);
    for (auto bodyIdx = 0ul; bodyIdx < m_model->nbodies; ++bodyIdx)
    {
      pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, m_model->nv);
      const auto &frameIdx = m_bodyCoMFrames[bodyIdx];
      pinocchio::getFrameJacobian(*m_model, *m_data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, J);
      Eigen::VectorXd dg = J * dq;
      accessor_dgdq_w[bodyIdx] = sofa::rigidbodydynamics::vec6ToSofaType<Eigen::VectorXd>(dg);
    }
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJT(
      const core::MechanicalParams *mparams, 
      DataVecDeriv_t<In>& out,
      DataVecDeriv_t<InRoot>* outRoot,
      const DataVecDeriv_t<Out>& in)
  {
    SOFA_UNUSED(mparams);

    assert(m_model);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    // maps spatial forces applied on each body to torques on joints

    // out will be the resulting torque on each joint
    helper::WriteAccessor<DataVecDeriv_t<In>> outTorquesWa(out);
    // size = nv if no root joint, nv+6 if using free-flyer root joint

    // in is a vector of spatial forces for each body
    helper::ReadAccessor<DataVecDeriv_t<Out>> inWrenchRa(in);
    // size = nbodies
    // msg_info() << "input bodies forces: ra_wrench_w = " << ra_wrench_w;

    Eigen::VectorXd jointsTorques = Eigen::VectorXd::Zero(m_model->nv);
    for (auto bodyIdx = 0ul; bodyIdx < m_model->nbodies; ++bodyIdx)
    {
      // bodyForce is a spatial force so 6-vector
      const Eigen::VectorXd bodyForce = sofa::rigidbodydynamics::vectorToEigen(inWrenchRa[bodyIdx], 6);
      pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, m_model->nv);
      const auto &frameIdx = m_bodyCoMFrames[bodyIdx];

      pinocchio::getFrameJacobian(*m_model, *m_data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, J);
      jointsTorques += J.transpose() * bodyForce;
    }

    if (m_fromRootModel and outRoot != nullptr)
    {
      // joint torques contains first 6 parameters for the root joint, and nv-6 parameters for other joints
      helper::WriteAccessor<DataVecDeriv_t<InRoot>> outRootWrenchWa(outRoot);
      // msg_info() << "wa_rootWrench_w size = " << wa_rootWrench_w.size();
      // write (add) root joint spatial force
      outRootWrenchWa[0] += sofa::rigidbodydynamics::vec6ToSofaType(jointsTorques.head<6>());
      // write (add) joint torques
      for (auto i = 0ul; i < jointsTorques.size() - 6; ++i)
      {
        outTorquesWa[i][0] += jointsTorques[i + 6];
      }
    }
    else
    {
      // no root joints case, only write (add) joint torques
      for (auto i = 0ul; i < jointsTorques.size(); ++i)
      {
        outTorquesWa[i][0] += jointsTorques[i];
      }
    }
    // msg_info() << "********* END KinematicChainMapping applyJT";
  }

  template <class TIn, class TInRoot, class TOut>
  void KinematicChainMapping<TIn, TInRoot, TOut>::applyJT(
      const core::ConstraintParams *cparams, 
      DataMatrixDeriv_t<In>& out,
      DataMatrixDeriv_t<InRoot>* outRoot,
      const DataMatrixDeriv_t<Out>& in)
  {
    SOFA_UNUSED(cparams);

    assert(m_model);

    if (d_componentState.getValue() == sofa::core::objectmodel::ComponentState::Invalid)
      return;

    // out will be the resulting torque on each joint
    helper::WriteAccessor<DataMatrixDeriv_t<In>> outTorquesWa(out);

    // dataMatInConst
    helper::ReadAccessor<DataMatrixDeriv_t<Out>> inWrenchRa(in);

    // row = constraint
    for (auto rowIt = inWrenchRa->begin(); rowIt != inWrenchRa->end(); ++rowIt)
    {
      Eigen::VectorXd jointsTorques = Eigen::VectorXd::Zero(m_model->nv);
      // col = dof (bodies)
      for (auto colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
      {
        // retrieve body frame index (centered at CoM) for each body
        const auto bodyIdx = colIt.index();
        const auto &frameIdx = m_bodyCoMFrames[bodyIdx];

        // get jacobian associated to this body
        pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, m_model->nv);
        pinocchio::getFrameJacobian(*m_model, *m_data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, J);

        // sum joint torques for all bodies
        jointsTorques += J.transpose() * sofa::rigidbodydynamics::spatialVelocityToEigen(colIt.val());
      }

      if (m_fromRootModel and outRoot != nullptr)
      {
        // write (add) root joint spatial force
        helper::WriteAccessor<DataMatrixDeriv_t<InRoot>> outRootWrenchWa(outRoot);
        auto oRoot = outRootWrenchWa->writeLine(rowIt.index());
        const Deriv_t<InRoot> rootWrench(sofa::rigidbodydynamics::vec6ToSofaType(jointsTorques.head<6>()));
        oRoot.addCol(0, rootWrench);
        // write summed joint torques for this constraint to output
        auto outRowIt = outTorquesWa->writeLine(rowIt.index());
        for (auto i = 0ul; i < jointsTorques.size() - 6; ++i)
        {
          const Deriv_t<In> torque(jointsTorques[i + 6]); // InDeriv is Vec1 type
          outRowIt.addCol(i, torque);
        }
      }
      else
      {
        // write summed joint torques for this constraint to output
        auto outRowIt = outTorquesWa->writeLine(rowIt.index());
        for (auto i = 0ul; i < jointsTorques.size(); ++i)
        {
          const Deriv_t<In> torque(jointsTorques[i]); // InDeriv is Vec1 type
          outRowIt.addCol(i, torque);
        }
      }
    }

    // print output matrices
    // if (m_fromRootModel and not dataMatOut2Const.empty())
    // {
    //   msg_info() << "outRootWrench CRS matrix:";
    //   InRootDataMatrixDeriv *outRootWrench = dataMatOut2Const[0];
    //   helper::WriteAccessor<InRootDataMatrixDeriv> outRootWrenchWa(outRootWrench);
    //   for (auto rowIt = outRootWrenchWa->begin(); rowIt != outRootWrenchWa->end(); ++rowIt)
    //   {
    //     for (auto colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
    //     {
    //       msg_info() << "row[" << rowIt.index() << "], col[" << colIt.index() << "]:" << colIt.val();
    //     }
    //   }
    // }
    // msg_info() << "outTorques CRS matrix:";
    // for (auto rowIt = outTorquesWa->begin(); rowIt != outTorquesWa->end(); ++rowIt)
    // {
    //   for (auto colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
    //   {
    //     msg_info() << "row[" << rowIt.index() << "], col[" << colIt.index() << "]:" << colIt.val();
    //   }
    // }

    // msg_info() << "********* END KinematicChainMapping applyJT On MatrixDeriv";
  }

} // namespace sofa::component::mapping::nonlinear
