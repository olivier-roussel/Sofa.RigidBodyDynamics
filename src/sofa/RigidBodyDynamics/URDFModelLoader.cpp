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
#include <sofa/RigidBodyDynamics/URDFModelLoader.h>

#include <sofa/RigidBodyDynamics/Conversions.h>
#include <sofa/RigidBodyDynamics/GeometryConversions.h>
#include <sofa/RigidBodyDynamics/KinematicChainMapping.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/component/mapping/nonlinear/RigidMapping.h>
#include <sofa/component/mass/UniformMass.h>
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/component/solidmechanics/spring/RestShapeSpringsForceField.h>
// #include <sofa/component/mechanicalload/ConstantForceField.h>

// for testing constraints
#include <sofa/component/constraint/lagrangian/model/BilateralLagrangianConstraint.h>
#include <sofa/component/constraint/lagrangian/solver/GenericConstraintSolver.h>
#include <sofa/component/linearsolver/direct/SparseLDLSolver.h>
#include <sofa/component/odesolver/backward/EulerImplicitSolver.h>
#include <sofa/component/constraint/lagrangian/correction/GenericConstraintCorrection.h>

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/gl/component/rendering3d/OglModel.h>
#include <sofa/simulation/Node.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp> // XXX
#include <hpp/fcl/collision_object.h>         // XXX

using namespace sofa::defaulttype;

using MechanicalObjectVec1 = sofa::component::statecontainer::MechanicalObject<Vec1Types>;
using MechanicalObjectRigid3 = sofa::component::statecontainer::MechanicalObject<Rigid3Types>;

namespace sofa::rigidbodydynamics
{

  URDFModelLoader::URDFModelLoader()
      : d_urdfFilename(initData(&d_urdfFilename, "urdfFilename", "Filename of the URDF model")), 
      d_modelDirectory(initData(&d_modelDirectory, "modelDirectory", "Directory containing robot models")),
      d_useFreeFlyerRootJoint(initData(&d_useFreeFlyerRootJoint, false, "useFreeFlyerRootJoint", "True if root joint is a Free Flyer joint, false if none")),
      d_q0(initData(&d_q0, "q0", "Default configuration values of robot DoFs"))
  {
    //     addUpdateCallback("updateURDFSources", {&d_urdfFilename, &d_modelDirectory}, [this](const core::DataTracker& )
    // {
    //     reinit();
    //     return sofa::core::objectmodel::ComponentState::Valid;
    // }, {&d_positions});
  }

  void URDFModelLoader::setURDFFilename(const std::string &f)
  {
    d_urdfFilename.setValue(f);
  }

  const std::string &URDFModelLoader::getURDFFilename()
  {
    return d_urdfFilename.getValue();
  }

  void URDFModelLoader::setModelDirectory(const std::string &f)
  {
    d_modelDirectory.setValue(f);
  }

  const std::string &URDFModelLoader::getModelDirectory()
  {
    return d_modelDirectory.getValue();
  }

  void URDFModelLoader::setUseFreeFlyerRootJoint(bool useFreeFlyerRootJoint)
  {
    d_useFreeFlyerRootJoint.setValue(useFreeFlyerRootJoint);
  }

  bool URDFModelLoader::getUseFreeFlyerRootJoint()
  {
    return d_useFreeFlyerRootJoint.getValue();
  }

  void URDFModelLoader::init()
  {
    this->reinit();
  }

  void URDFModelLoader::reinit()
  {
    const std::string &urdfFilename = d_urdfFilename.getValue();
    const std::string &modelDir = d_modelDirectory.getValue();
    const bool useFreeFlyerRootJoint = d_useFreeFlyerRootJoint.getValue();

    msg_info() << " Loading robot from URDF file: " << urdfFilename;
    msg_info() << "Model directory: " << modelDir;
    std::shared_ptr<pinocchio::Model> model;
    // std::shared_ptr<pinocchio::GeometryModel> collisionModel;
    std::shared_ptr<pinocchio::GeometryModel> visualModel;
    std::vector<pinocchio::FrameIndex> bodyCoMFrames;

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node

    // clear robot scene tree
    {
      const auto jointsNode = context->getChild("Joints");
      if (jointsNode)
      {
        context->removeChild(jointsNode);
        msg_info() << "Joints node was already present in robot scene tree. Removing it...";
      }
      const auto rootJointNode = context->getChild("RootJoint");
      if (rootJointNode)
      {
        context->removeChild(rootJointNode);
        msg_info() << "RootJoint node was already present in robot scene tree. Removing it...";
      }
      const auto bodiesNode = context->getChild("Bodies");
      if (bodiesNode)
      {
        context->removeChild(bodiesNode);
        msg_info() << "Bodies node was already present in robot scene tree. Removing it...";
      }
    }

    // load URDF model
    try
    {
      model = std::make_shared<pinocchio::Model>();
      if(useFreeFlyerRootJoint)
      {
        pinocchio::urdf::buildModel(urdfFilename, pinocchio::JointModelFreeFlyer(), *model);
        msg_info() << "Built robot model (with Free Flyer root joint) from URDF file: " << urdfFilename;
      }
      else
      {
        pinocchio::urdf::buildModel(urdfFilename, *model);
        msg_info() << "Built robot model (with fixed root joint) from URDF file: " << urdfFilename;
      }
      msg_info() << "Robot nq = " << model->nq << " / nv = " << model->nv;
      msg_info() << "Robot njoints = " << model->njoints << " / nbodies = " << model->nbodies << " / nframes = " << model->nframes;
      msg_info() << "Robot model 6d gravity g = " << model->gravity;
      msg_info() << "Robot inertias vector size = " << model->inertias.size();

      for(auto jointIdx = 0u; jointIdx < model->njoints; ++jointIdx)
      {
        msg_info() << "Joint[" << jointIdx << "]: " << model->names[jointIdx] << " / " << model->joints[jointIdx];
      }

      // collisionModel = std::make_shared<pinocchio::GeometryModel>();
      // pinocchio::urdf::buildGeom(*model, urdfFilename, pinocchio::COLLISION, *collisionModel, modelDir);
      // msg_info() << "Built robot collision model from URDF file: " << urdf_filename;

      visualModel = std::make_shared<pinocchio::GeometryModel>();
      pinocchio::urdf::buildGeom(*model, urdfFilename, pinocchio::VISUAL, *visualModel, modelDir);
      // msg_info() << "Built robot visual model from URDF file: " << urdf_filename;

      // add a frame for each body centered on its CoM and that will be used as body DoF by SOFA
      bodyCoMFrames.reserve(model->nbodies);
      for (pinocchio::JointIndex bodyIdx = 0; bodyIdx < model->nbodies; ++bodyIdx)
      {
        const pinocchio::SE3 bodyCoM_i = pinocchio::SE3(Eigen::Matrix3d::Identity(), model->inertias[bodyIdx].lever());
        const auto bodyFrameCoM = pinocchio::Frame{"Body_" + std::to_string(bodyIdx) + "_CoM", bodyIdx, bodyCoM_i, pinocchio::FrameType::OP_FRAME};
        bodyCoMFrames.push_back(model->addFrame(bodyFrameCoM));
      }
    }
    catch (std::exception &e)
    {
      msg_error() << "Caught exception: " << e.what();
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    // create robot scene tree
    const auto jointsNode = context->createChild("Joints");

    const auto jointsDofs = New<MechanicalObjectVec1>();
    jointsDofs->setName("dofs");
    auto nqWithoutRootJoint = useFreeFlyerRootJoint ? model->nq - 7 : model->nq;
    msg_info() << "nqWithoutRootJoint = " << nqWithoutRootJoint;
    jointsDofs->resize(nqWithoutRootJoint);
    // set desired position specified from \"q0\" data field
    sofa::type::Vec1d defaultDofValue;
    defaultDofValue.set(0.);
    sofa::type::vector<sofa::type::Vec1d> q0Values(nqWithoutRootJoint, defaultDofValue);
    // TODO retrieve default configs q0 from URDF if any
    d_q0.setValue(q0Values);
    jointsDofs->x0.setParent(&d_q0);
    jointsNode->addObject(jointsDofs);

    // spring to control robot dofs to desired dofs set by rest position (x0)
    const auto restShapeForceField = New<sofa::component::solidmechanics::spring::RestShapeSpringsForceField<Vec1Types>>();
    restShapeForceField->setName("RestShapeSpringsForceField");
    restShapeForceField->d_stiffness.setValue({1.e3});
    sofa::type::vector<sofa::Index> pointsIndexes;
    for (sofa::Index idx = 0ul; idx < nqWithoutRootJoint; ++idx)
    {
      pointsIndexes.push_back(idx);
    }
    restShapeForceField->d_points.setValue(pointsIndexes);
    jointsNode->addObject(restShapeForceField);

    const auto bodiesNode = jointsNode->createChild("Bodies");

    // create mapping between robot joints dofs and its bodies placements
    const auto kinematicChainMapping = New<sofa::component::mapping::KinematicChainMapping<Vec1Types, Rigid3Types, Rigid3Types>>();
    kinematicChainMapping->setName("kinematicChainMapping");
    kinematicChainMapping->setBodyCoMFrames(bodyCoMFrames);
    kinematicChainMapping->f_printLog.setValue(true);
    kinematicChainMapping->setModel(model);
    // kinematicChainMapping->setCollisionModel(collisionModel);
    kinematicChainMapping->setVisualModel(visualModel);
    // set mapping input1
    kinematicChainMapping->addInputModel1(jointsDofs.get());


    // one dof container for all bodies version
    const auto bodiesDof = New<MechanicalObjectRigid3>();
    bodiesDof->setName("bodiesDofs");
    bodiesDof->resize(model->nbodies);
    bodiesNode->addObject(bodiesDof);

    kinematicChainMapping->addOutputModel(bodiesDof.get());
    bodiesNode->addObject(kinematicChainMapping);

    // set mapping input2: free flyer root joint if any specified
    if(useFreeFlyerRootJoint)
    {
      const auto rootJointNode = context->createChild("RootJoint");

      const auto rootJointDof = New<MechanicalObjectRigid3>();
      rootJointDof->setName("Free-Flyer");
      rootJointDof->resize(1);
      rootJointNode->addObject(rootJointDof);
      
      const auto rootRestShapeForceField = New<sofa::component::solidmechanics::spring::RestShapeSpringsForceField<Rigid3Types>>();
      rootRestShapeForceField->setName("root joint spring force field");
      rootRestShapeForceField->d_stiffness.setValue({1.e3});
      rootRestShapeForceField->d_angularStiffness.setValue({1.e3});
      rootRestShapeForceField->d_points.setValue({0});
      rootJointNode->addObject(rootRestShapeForceField);

      // Bodies node have two parents: rootJointNode and jointsNode
      rootJointNode->addChild(bodiesNode);

      // set mapping input2
      kinematicChainMapping->addInputModel2(rootJointDof.get());
    }

    // Testing constraints (applytJT with constraint matrices)
    simulation::Node *rootNode = dynamic_cast<simulation::Node *>(this->getContext()->getRootContext()); // access to root node
    const auto dummyNode = rootNode->createChild("dummyNode");

    const auto constraintSolver = New<sofa::component::linearsolver::direct::SparseLDLSolver< sofa::linearalgebra::CompressedRowSparseMatrix< sofa::type::Mat<3,3,SReal> >,sofa::linearalgebra::FullVector<SReal> >>();
    constraintSolver->setName("SparseLDLSolver_constraint");
    dummyNode->addObject(constraintSolver);

    const auto implicitSolver = New<sofa::component::odesolver::backward::EulerImplicitSolver>();
    implicitSolver->setName("EulerImplicitSolver_constraint");
    dummyNode->addObject(implicitSolver);

    const auto genericConstraintCorrection = New<sofa::component::constraint::lagrangian::correction::GenericConstraintCorrection>();
    genericConstraintCorrection->setName("GenericConstraintCorrection_constraint");
    dummyNode->addObject(genericConstraintCorrection);

    const auto mecDummyObject = New<sofa::component::statecontainer::MechanicalObject<Vec3Types>>();
    mecDummyObject->setName("mecDummyObject");
    mecDummyObject->setTranslation(-0.19, -0.137099, -0.262249);
    dummyNode->addObject(mecDummyObject);

    const auto restShapeForceFieldDummy = New<sofa::component::solidmechanics::spring::RestShapeSpringsForceField<Vec3Types>>();
    restShapeForceFieldDummy->setName("restShapeForceField_constraint");
    restShapeForceFieldDummy->d_points.setValue({0});
    restShapeForceFieldDummy->d_stiffness.setValue({1.e3});
    dummyNode->addObject(restShapeForceFieldDummy);

    const auto bodyMassDummy = New<sofa::component::mass::UniformMass<Vec3Types>>();
    bodyMassDummy->setName("mass_constraint");
    bodyMassDummy->setTotalMass(1.);
    dummyNode->addObject(bodyMassDummy);

    auto rootGenericConstraintSolverObj = rootNode->getObject("GenericConstraintSolver");
    if(rootGenericConstraintSolverObj)
    {
      const auto rootGenericConstraintSolver = dynamic_cast<sofa::component::constraint::lagrangian::solver::GenericConstraintSolver *>(rootGenericConstraintSolverObj);
      if(rootGenericConstraintSolver)
      {
        msg_info() << "Successfully found back object GenericConstraintSolver";
        rootGenericConstraintSolver->init();
        // rootGenericConstraintSolver->l_constraintCorrections.add(genericConstraintCorrection);
        // genericConstraintCorrection->addConstraintSolver(rootGenericConstraintSolver);
      }
      else
      {
        msg_error() << "Failed to cast object GenericConstraintSolver";
      }
    }
    else
    {
      msg_error() << "Failed to find object GenericConstraintSolver";
    }

    // ------ dummyNode

    const auto constraintNode = bodiesNode->createChild("Constraints");
    const auto mecObject = New<sofa::component::statecontainer::MechanicalObject<Vec3Types>>();
    mecObject->setName("mecObject");
    mecObject->setTranslation(0., 0., 0.);
    constraintNode->addObject(mecObject);

    const auto constraint = New<sofa::component::constraint::lagrangian::model::BilateralLagrangianConstraint<Vec3Types>>(mecObject.get(), mecDummyObject.get());
    const auto nullPoint = Vec3Types::Coord(0., 0., 0.);
    constraint->addContact(Vec3Types::Deriv(), nullPoint, nullPoint, 0., 0, 0, nullPoint, nullPoint, 0l, sofa::component::constraint::lagrangian::model::BilateralLagrangianConstraint<Vec3Types>::BilateralLagrangianConstraint::PersistentID());
    constraint->setName("constraint");
    constraintNode->addObject(constraint);

    const auto constraintMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Vec3Types>>();
    constraintMapping->setModels(bodiesDof.get(), mecObject.get());
    constraintMapping->d_index = model->nbodies -1; // apply on last body
    constraintNode->addObject(constraintMapping);


    for (pinocchio::JointIndex bodyIdx = 0; bodyIdx < model->nbodies; ++bodyIdx)
    {
      const auto bodyNode = bodiesNode->createChild("Body_" + std::to_string(bodyIdx));

      const auto& bodyInertia = model->inertias[bodyIdx];

      const auto bodyRigid = New<MechanicalObjectRigid3>();
      bodyRigid->setName("bodyRigid");
      const Eigen::Vector3d invBodyCoMTranslation = -bodyInertia.lever();
      bodyRigid->setTranslation(invBodyCoMTranslation.x(), invBodyCoMTranslation.y(), invBodyCoMTranslation.z());
      bodyNode->addObject(bodyRigid);

      const auto bodyMass = New<sofa::component::mass::UniformMass<Rigid3Types>>();
      bodyMass->setName("mass");
      sofa::defaulttype::Rigid3dMass rigidMass;
      rigidMass.mass = bodyInertia.mass();
      rigidMass.inertiaMatrix = sofa::rigidbodydynamics::mat3ToSofaType(bodyInertia.inertia().matrix());
      rigidMass.volume = 1.; // XXX: should not be used here as we only deal with rigid bodies, so we should be able to set any value
      rigidMass.recalc();
      bodyMass->setMass(rigidMass);
      bodyNode->addObject(bodyMass);

      const auto bodyMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Rigid3Types>>();
      bodyMapping->setName("bodyMapping");
      bodyMapping->setModels(bodiesDof.get(), bodyRigid.get());
      bodyMapping->d_index = bodyIdx;
      bodyMapping->d_globalToLocalCoords = false;
      bodyNode->addObject(bodyMapping);

      // add visual body node
      const auto visualNode = bodyNode->createChild("Visual");

      // get joint associated geometries
      const auto visualGeomIndexesIt = kinematicChainMapping->visualData()->innerObjects.find(bodyIdx);
      if (visualGeomIndexesIt != kinematicChainMapping->visualData()->innerObjects.end())
      {
        for (const auto &geomIdx : visualGeomIndexesIt->second)
        {
          const auto &geom = visualModel->geometryObjects[geomIdx];

          const auto visualBodyNode = visualNode->createChild(geom.name);
          // msg_info() << "body[" << bodyIdx << "]:geom name: " << geom.name << " / parent joint: " << geom.parentJoint << " / object type: " << static_cast<int>(geom.geometry->getObjectType()) << " / node type: " << static_cast<int>(geom.geometry->getNodeType());
          // msg_info() << "overrideMaterial: " << geom.overrideMaterial << " / mesh color: " << geom.meshColor;
          // msg_info() << "meshPath: " << geom.meshPath;
          // msg_info() << "meshTexturePath: " << geom.meshTexturePath;
          // msg_info() << "meshMaterial: " << typeid(geom.meshMaterial).name();

          auto visualBodyMesh = sofa::rigidbodydynamics::fclGeometryToSofaTopology(geom.geometry, geom.placement, geom.meshScale);
          if (not visualBodyMesh)
          {
            msg_error() << "Failed to convert pinocchio FCL geometry to Sofa MeshTopology";
            msg_error() << "FCL geometry object type: " << static_cast<int>(geom.geometry->getObjectType()) << ", FCL geometry node type: " << static_cast<int>(geom.geometry->getNodeType());
            d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
            return;
          }
          visualBodyNode->addObject(visualBodyMesh);

          // add visual openGL model
          auto visualBodyModel = New<sofa::gl::component::rendering3d::OglModel>();
          visualBodyModel->setName("visualModel");
          visualBodyModel->l_topology = visualBodyMesh;
          visualBodyModel->init();
          visualBodyModel->initVisual();
          visualBodyModel->updateVisual();
          visualBodyNode->addObject(visualBodyModel);

          const auto visualMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Vec3Types>>();
          visualMapping->setName("visualMapping");
          visualMapping->setModels(bodyRigid.get(), visualBodyModel.get());
          visualMapping->f_mapConstraints.setValue(false);
          visualMapping->f_mapForces.setValue(false);
          visualMapping->f_mapMasses.setValue(false);
          visualBodyNode->addObject(visualMapping);
        }
      }
    }



    msg_info() << "Model has " << model->referenceConfigurations.size() << " reference configurations registered";
  }

} /// namespace sofa::rigidbodydynamics

int URDFModelLoaderClass = sofa::core::RegisterObject("Loads robot from URDF file.").add<sofa::rigidbodydynamics::URDFModelLoader>();