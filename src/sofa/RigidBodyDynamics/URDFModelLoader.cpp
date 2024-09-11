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
#include <sofa/component/solidmechanics/spring/RestShapeSpringsForceField.h> // TODO remove from here and move to external scene

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/gl/component/rendering3d/OglModel.h>
#include <sofa/simulation/Node.h>

#include <pinocchio/parsers/urdf.hpp>

using namespace sofa::defaulttype;

using MechanicalObjectVec1 = sofa::component::statecontainer::MechanicalObject<Vec1Types>;
using MechanicalObjectRigid3 = sofa::component::statecontainer::MechanicalObject<Rigid3Types>;

namespace sofa::rigidbodydynamics
{

  URDFModelLoader::URDFModelLoader() : sofa::core::loader::SceneLoader() ,
      d_modelDirectory(initData(&d_modelDirectory, "modelDirectory", "Directory containing robot models")),
      d_useFreeFlyerRootJoint(initData(&d_useFreeFlyerRootJoint, false, "useFreeFlyerRootJoint", "True if root joint is a Free Flyer joint, false if none")),
      d_q0(initData(&d_q0, "q0", "Default configuration values of robot DoFs"))
  {
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

  void URDFModelLoader::reinit()
  {
    // no op
  }

  void URDFModelLoader::init()
  {
    // no op
  }

  bool URDFModelLoader::load()
  {
    const std::string &urdfFilename = getFilename();
    const std::string &modelDir = d_modelDirectory.getValue();
    const bool useFreeFlyerRootJoint = d_useFreeFlyerRootJoint.getValue();

    msg_info() << "Loading robot from URDF file: " << urdfFilename;
    msg_info() << "Model directory: " << modelDir;
    std::shared_ptr<pinocchio::Model> model;
    std::shared_ptr<pinocchio::GeometryModel> collisionModel;
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

      // TODO use collisionModel to create collision nodes in SOFA
      collisionModel = std::make_shared<pinocchio::GeometryModel>();
      pinocchio::urdf::buildGeom(*model, urdfFilename, pinocchio::COLLISION, *collisionModel, modelDir);
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
      return false;
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
    // TODO retrieve default configs q0 from SRDF file if any
    defaultDofValue.set(0.);
    sofa::type::vector<sofa::type::Vec1d> q0Values(nqWithoutRootJoint, defaultDofValue);
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
    const auto kinematicChainMapping = New<sofa::component::mapping::nonlinear::KinematicChainMapping<Vec1Types, Rigid3Types, Rigid3Types>>();
    kinematicChainMapping->setName("kinematicChainMapping");
    kinematicChainMapping->setBodyCoMFrames(bodyCoMFrames);
    kinematicChainMapping->f_printLog.setValue(true);
    kinematicChainMapping->setModel(model);

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

    msg_info() << "-- model->nbodies" << model->nbodies;
    msg_info() << "-- model->njoints" << model->njoints;

    for (pinocchio::JointIndex jointIdx = 0; jointIdx < model->njoints; ++jointIdx)
    {
      msg_info() << "-- joint " << jointIdx;
      msg_info() << "-- joint name " << model->names[jointIdx];
      const auto jointNode = bodiesNode->createChild(model->names[jointIdx]);

      const auto& jointInertia = model->inertias[jointIdx];

      const auto bodyRigid = New<MechanicalObjectRigid3>();
      bodyRigid->setName("jointRigid");
      const Eigen::Vector3d invBodyCoMTranslation = -jointInertia.lever();
      bodyRigid->setTranslation(invBodyCoMTranslation.x(), invBodyCoMTranslation.y(), invBodyCoMTranslation.z());
      jointNode->addObject(bodyRigid);

      const auto bodyMass = New<sofa::component::mass::UniformMass<Rigid3Types>>();
      bodyMass->setName("mass");
      sofa::defaulttype::Rigid3dMass rigidMass;
      rigidMass.mass = jointInertia.mass();
      msg_info() << "-- mass = " << jointInertia.mass();

      rigidMass.inertiaMatrix = sofa::rigidbodydynamics::mat3ToSofaType(jointInertia.inertia().matrix());
      rigidMass.volume = 1.; // XXX: should not be used here as we only deal with rigid bodies, so we should be able to set any value
      rigidMass.recalc();
      bodyMass->setMass(rigidMass);
      jointNode->addObject(bodyMass);

      const auto bodyMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Rigid3Types>>();
      bodyMapping->setName("jointMapping");
      bodyMapping->setModels(bodiesDof.get(), bodyRigid.get());
      bodyMapping->d_index = jointIdx;
      bodyMapping->d_globalToLocalCoords = false;
      jointNode->addObject(bodyMapping);

      // add visual body node
      const auto visualNode = jointNode->createChild("Visual");

      // get joint associated geometries
      const auto visualData = std::make_shared<pinocchio::GeometryData>(*visualModel);
      const auto visualGeomIndexesIt = visualData->innerObjects.find(jointIdx);
      if (visualGeomIndexesIt != visualData->innerObjects.end())
      {
        for (const auto &geomIdx : visualGeomIndexesIt->second)
        {
          const auto &geom = visualModel->geometryObjects[geomIdx];

          const auto visualBodyNode = visualNode->createChild(geom.name);
          // msg_info() << "joint[" << jointIdx << "]:geom name: " << geom.name << " / parent joint: " << geom.parentJoint << " / object type: " << static_cast<int>(geom.geometry->getObjectType()) << " / node type: " << static_cast<int>(geom.geometry->getNodeType());
          // msg_info() << "overrideMaterial: " << geom.overrideMaterial << " / mesh color: " << geom.meshColor;
          // msg_info() << "meshPath: " << geom.meshPath;
          // msg_info() << "meshTexturePath: " << geom.meshTexturePath;

          auto visualBodyMesh = sofa::rigidbodydynamics::fclGeometryToSofaTopology(geom.geometry, geom.placement, geom.meshScale);
          if (not visualBodyMesh)
          {
            msg_error() << "Failed to convert pinocchio FCL geometry to Sofa MeshTopology";
            msg_error() << "FCL geometry object type: " << static_cast<int>(geom.geometry->getObjectType()) << ", FCL geometry node type: " << static_cast<int>(geom.geometry->getNodeType());
            continue;
          }
          visualBodyNode->addObject(visualBodyMesh);

          // add visual openGL model
          auto visualBodyModel = New<sofa::gl::component::rendering3d::OglModel>();
          visualBodyModel->f_printLog = true;
          visualBodyModel->setName("visualModel");
          visualBodyModel->l_topology = visualBodyMesh;
          visualBodyModel->setColor(geom.meshColor[0], geom.meshColor[1], geom.meshColor[2], geom.meshColor[3]);
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

    return true;
  }

} /// namespace sofa::rigidbodydynamics

int URDFModelLoaderClass = sofa::core::RegisterObject("Loads robot from URDF file.").add<sofa::rigidbodydynamics::URDFModelLoader>();