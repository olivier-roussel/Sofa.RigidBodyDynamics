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

#include <sofa/core/Multi2Mapping.h>

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/simulation/Node.h>

#include <pinocchio/multibody/model.hpp>

namespace sofa::component::mapping::nonlinear
{

  template <class TIn, class TInRoot, class TOut>
  class SOFA_RIGIDBODYDYNAMICS_API KinematicChainMapping : public core::Multi2Mapping<TIn, TInRoot, TOut>
  {

  public:
    SOFA_CLASS(SOFA_TEMPLATE3(KinematicChainMapping, TIn, TInRoot, TOut), SOFA_TEMPLATE3(core::Multi2Mapping, TIn, TInRoot, TOut));

    using Inherit = core::Multi2Mapping<TIn, TInRoot, TOut>;
    using In = TIn;
    using InRoot = TInRoot;
    using Out = TOut;

    void apply(
      const core::MechanicalParams* mparams,
      const type::vector<DataVecCoord_t<Out> *>& dataVecOutPos,
      const type::vector<const DataVecCoord_t<In> *>& dataVecIn1Pos,
      const type::vector<const DataVecCoord_t<InRoot> *>& dataVecIn2Pos) override;

    void applyJ(
      const core::MechanicalParams *mparams,
      const type::vector<DataVecDeriv_t<Out> *> &dataVecOutVel,
      const type::vector<const DataVecDeriv_t<In> *> &dataVecIn1Vel,
      const type::vector<const DataVecDeriv_t<InRoot> *> &dataVecIn2Vel) override;

    void applyJT(
      const core::MechanicalParams *mparams,
      const type::vector<DataVecDeriv_t<In> *> &dataVecOut1Force,
      const type::vector<DataVecDeriv_t<InRoot> *> &dataVecOut2Force,
      const type::vector<const DataVecDeriv_t<Out> *> &dataVecInForce) override;

    void applyJT(
      const core::ConstraintParams * cparams,
      const type::vector<DataMatrixDeriv_t<In> *> &dataMatOut1Const,
      const type::vector<DataMatrixDeriv_t<InRoot> *> &dataMatOut2Const,
      const type::vector<const DataMatrixDeriv_t<Out> *> &dataMatInConst) override;

    void applyDJT(
      const core::MechanicalParams * /*mparams*/,
      core::MultiVecDerivId /*inForce*/,
      core::ConstMultiVecDerivId /*outForce*/) override
    {
      // no op
    }

    const sofa::linearalgebra::BaseMatrix *getJ() override { return nullptr; }

    void init() override;
    void bwdInit() override;
    void reset() override;
    void draw(const core::visual::VisualParams *vparams) override;

    void setModel(const std::shared_ptr<pinocchio::Model> &model);

    void setBodyCoMFrames(const std::vector<pinocchio::FrameIndex>& bodyCoMFrames);

  protected:
    KinematicChainMapping();
    ~KinematicChainMapping() override
    {
      // no op
    }

  private:
    std::shared_ptr<pinocchio::Model> m_model;
    std::shared_ptr<pinocchio::Data> m_data;
    std::vector<pinocchio::FrameIndex> m_bodyCoMFrames;
    core::State<InRoot>* m_fromRootModel;

    using core::Multi2Mapping<TIn, TInRoot, TOut>::d_componentState;
    Data<sofa::Index> d_indexFromRoot; ///< Corresponding index if the base of the articulated system is attached to input2. Default is last index.

    void checkIndexFromRoot();

    void apply(const core::MechanicalParams* mparams,
      DataVecCoord_t<Out>& out,
      const DataVecCoord_t<In>& in,
      const DataVecCoord_t<InRoot>* inRoot);

    void applyJ(
      const core::MechanicalParams* mparams,
      DataVecDeriv_t<Out>& out,
      const DataVecDeriv_t<In>& in,
      const DataVecDeriv_t<InRoot>* inRoot);

    void applyJT(
      const core::MechanicalParams *mparams,
      DataVecDeriv_t<In>& out,
      DataVecDeriv_t<InRoot>* outRoot,
      const DataVecDeriv_t<Out>& in);

    void applyJT(
      const core::ConstraintParams *mparams,
      DataMatrixDeriv_t<In>& out,
      DataMatrixDeriv_t<InRoot>* outRoot,
      const DataMatrixDeriv_t<Out>& in);
  };

#if !defined(SOFA_COMPONENT_MAPPING_KINEMATICCHAINMAPPING_CPP)

  extern template class SOFA_RIGIDBODYDYNAMICS_API KinematicChainMapping<sofa::defaulttype::Vec1Types, sofa::defaulttype::Rigid3Types, sofa::defaulttype::Rigid3Types>;

#endif

} // namespace sofa::component::mapping::nonlinear
