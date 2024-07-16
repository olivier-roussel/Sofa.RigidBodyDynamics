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
#include <pinocchio/multibody/geometry.hpp>

namespace sofa::component::mapping
{

  template <class TIn, class TInRoot, class TOut>
  class SOFA_RIGIDBODYDYNAMICS_API KinematicChainMapping : public core::Multi2Mapping<TIn, TInRoot, TOut>
  {

  public:
    SOFA_CLASS(SOFA_TEMPLATE3(KinematicChainMapping, TIn, TInRoot, TOut), SOFA_TEMPLATE3(core::Multi2Mapping, TIn, TInRoot, TOut));

    typedef core::Multi2Mapping<TIn, TInRoot, TOut> Inherit;
    typedef TIn In;
    typedef TInRoot InRoot;
    typedef TOut Out;

    typedef typename In::Coord InCoord;
    typedef typename In::Deriv InDeriv;
    typedef typename In::VecCoord InVecCoord;
    typedef typename In::VecDeriv InVecDeriv;

    typedef typename InRoot::Coord InRootCoord;
    typedef typename InRoot::Deriv InRootDeriv;
    typedef typename InRoot::VecCoord InRootVecCoord;
    typedef typename InRoot::VecDeriv InRootVecDeriv;

    typedef typename Out::Coord OutCoord;
    typedef typename Out::Deriv OutDeriv;
    typedef typename Out::VecCoord OutVecCoord;
    typedef typename Out::VecDeriv OutVecDeriv;

    typedef typename In::MatrixDeriv InMatrixDeriv;
    typedef Data<InVecCoord> InDataVecCoord;
    typedef Data<InVecDeriv> InDataVecDeriv;
    typedef Data<InMatrixDeriv> InDataMatrixDeriv;

    typedef typename InRoot::MatrixDeriv InRootMatrixDeriv;
    typedef Data<InRootVecCoord> InRootDataVecCoord;
    typedef Data<InRootVecDeriv> InRootDataVecDeriv;
    typedef Data<InRootMatrixDeriv> InRootDataMatrixDeriv;

    typedef typename Out::MatrixDeriv OutMatrixDeriv;
    typedef Data<OutVecCoord> OutDataVecCoord;
    typedef Data<OutVecDeriv> OutDataVecDeriv;
    typedef Data<OutMatrixDeriv> OutDataMatrixDeriv;

    void apply(
        const core::MechanicalParams *mparams, const type::vector<OutDataVecCoord *> &dataVecOutPos,
        const type::vector<const InDataVecCoord *> &dataVecIn1Pos,
        const type::vector<const InRootDataVecCoord *> &dataVecIn2Pos) override;
    void applyJ(
        const core::MechanicalParams *mparams, const type::vector<OutDataVecDeriv *> &dataVecOutVel,
        const type::vector<const InDataVecDeriv *> &dataVecIn1Vel,
        const type::vector<const InRootDataVecDeriv *> &dataVecIn2Vel) override;
    void applyJT(
        const core::MechanicalParams *mparams, const type::vector<InDataVecDeriv *> &dataVecOut1Force,
        const type::vector<InRootDataVecDeriv *> &dataVecOut2Force,
        const type::vector<const OutDataVecDeriv *> &dataVecInForce) override;
    void applyJT(
        const core::ConstraintParams * /*cparams*/, const type::vector<InDataMatrixDeriv *> & /* dataMatOut1Const */,
        const type::vector<InRootDataMatrixDeriv *> & /*dataMatOut2Const*/,
        const type::vector<const OutDataMatrixDeriv *> & /*dataMatInConst*/) override;

    void applyDJT(const core::MechanicalParams * /*mparams*/, core::MultiVecDerivId /*inForce*/, core::ConstMultiVecDerivId /*outForce*/) override
    {
      // no op
    }

    const sofa::linearalgebra::BaseMatrix *getJ() override { return nullptr; }

    void init() override;
    void bwdInit() override;
    void reset() override;
    void draw(const core::visual::VisualParams *vparams) override;

    void setModel(const std::shared_ptr<pinocchio::Model> &model);

    // void setCollisionModel(const std::shared_ptr<pinocchio::GeometryModel> &collisionModel);

    void setVisualModel(const std::shared_ptr<pinocchio::GeometryModel> &visualModel);

    void setBodyCoMFrames(const std::vector<pinocchio::FrameIndex>& bodyCoMFrames);

    // const std::shared_ptr<pinocchio::GeometryData>& collisionData() const;

    const std::shared_ptr<pinocchio::GeometryData>& visualData() const;

  protected:
    KinematicChainMapping();
    ~KinematicChainMapping() override
    {
      // no op
    }

  private:
    std::shared_ptr<pinocchio::Model> m_model;
    // std::shared_ptr<pinocchio::GeometryModel> m_collisionModel;
    std::shared_ptr<pinocchio::GeometryModel> m_visualModel;
    std::shared_ptr<pinocchio::Data> m_data;
    // std::shared_ptr<pinocchio::GeometryData> m_collisionData;
    std::shared_ptr<pinocchio::GeometryData> m_visualData;
    std::vector<pinocchio::FrameIndex> m_bodyCoMFrames;
    core::State<InRoot>* m_fromRootModel;

    using core::Multi2Mapping<TIn, TInRoot, TOut>::d_componentState;
    Data<sofa::Index> d_indexFromRoot; ///< Corresponding index if the base of the articulated system is attached to input2. Default is last index.

    void checkIndexFromRoot();
  };

#if !defined(SOFA_COMPONENT_MAPPING_KINEMATICCHAINMAPPING_CPP)

  extern template class SOFA_RIGIDBODYDYNAMICS_API KinematicChainMapping<sofa::defaulttype::Vec1Types, sofa::defaulttype::Rigid3Types, sofa::defaulttype::Rigid3Types>;

#endif

} // namespace sofa::component::mapping
