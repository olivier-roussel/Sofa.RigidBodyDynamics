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

#include <sofa/RigidBodyDynamics/CollisionPair.h>
#include <sofa/RigidBodyDynamics/Conversions.h>
#include <sofa/RigidBodyDynamics/GeometryConversions.h>
#include <sofa/RigidBodyDynamics/KinematicChainMapping.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/component/mapping/nonlinear/RigidMapping.h>
#include <sofa/component/mass/UniformMass.h>
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/component/collision/geometry/TriangleModel.h>
#include <sofa/component/collision/geometry/LineModel.h>
#include <sofa/component/collision/geometry/PointModel.h>

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
      d_addCollision(initData(&d_addCollision, true, "addCollision", "True if collision detection must be enabled for the robot (self-collision and other objects)")),
      d_qRest(initData(&d_qRest, "qRest", "Rest configuration values of robot DoFs")),
      d_qInit(initData(&d_qInit, "qInit", "Initial configuration values of robot DoFs"))
      // d_extraFramesNames(initData(&d_extraFramesNames, "extraFramesNames", "Optional frames names to add to kinematic mapping"))
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
    std::vector<pinocchio::FrameIndex> extraFrames;

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node

    // clear robot scene tree
    {
      const auto robotNode = context->getChild("Robot");
      if (robotNode)
      {
        context->removeChild(robotNode);
        msg_info() << "Robot node was already present in robot scene tree. Removing it...";
      }
      const auto rootJointNode = context->getChild("RootJoint");
      if (rootJointNode)
      {
        context->removeChild(rootJointNode);
        msg_info() << "RootJoint node was already present in robot scene tree. Removing it...";
      }
      const auto jointsNode = context->getChild("Joints");
      if (jointsNode)
      {
        context->removeChild(jointsNode);
        msg_info() << "Joints node was already present in robot scene tree. Removing it...";
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

      // msg_info() << "Adding all frames to kinematic mapping...";
      // for(auto frameIdx = 0u; frameIdx < model->nframes; ++frameIdx)
      // {
      //   msg_info() << "Frame[" << frameIdx << "]: " << model->frames[frameIdx].name <<" / parent Joint = " << model->frames[frameIdx].parentJoint << " / parent Frame = " << model->frames[frameIdx].parentFrame;
      //   extraFrames.push_back(frameIdx);
      // }

      // TODO use collisionModel to create collision nodes in SOFA
      collisionModel = std::make_shared<pinocchio::GeometryModel>();
      pinocchio::urdf::buildGeom(*model, urdfFilename, pinocchio::COLLISION, *collisionModel, modelDir);
      // msg_info() << "Built robot collision model from URDF file: " << urdf_filename;

      visualModel = std::make_shared<pinocchio::GeometryModel>();
      pinocchio::urdf::buildGeom(*model, urdfFilename, pinocchio::VISUAL, *visualModel, modelDir);
      // msg_info() << "Built robot visual model from URDF file: " << urdf_filename;

      // add a frame for each body centered on its CoM and that will be used as body DoF by SOFA
      bodyCoMFrames.reserve(model->njoints);
      for (pinocchio::JointIndex jointIdx = 0; jointIdx < model->njoints; ++jointIdx)
      {
        const pinocchio::SE3 bodyCoM_i = pinocchio::SE3(Eigen::Matrix3d::Identity(), model->inertias[jointIdx].lever());
        const auto bodyFrameCoM = pinocchio::Frame{"Body_" + std::to_string(jointIdx) + "_CoM", jointIdx, bodyCoM_i, pinocchio::FrameType::OP_FRAME};
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
    const auto robotNode = context->createChild("Robot");

    const auto jointsDofs = New<MechanicalObjectVec1>();
    jointsDofs->setName("dofs");
    auto nqWithoutRootJoint = useFreeFlyerRootJoint ? model->nq - 7 : model->nq;
    msg_info() << "nqWithoutRootJoint = " << nqWithoutRootJoint;

    msg_info() << "=========== d_qInit isSet = " << d_qInit.isSet();
    msg_info() << "=========== d_qRest isSet = " << d_qRest.isSet();
    if(not d_qInit.isSet())
    {
      sofa::type::Vec1d defaultDofValue;
      defaultDofValue.set(0.);
      sofa::type::vector<sofa::type::Vec1d> q0Values(nqWithoutRootJoint, defaultDofValue);
      d_qInit.setValue(q0Values);
    }
    if(not d_qRest.isSet())
    {
      sofa::type::Vec1d defaultDofValue;
      defaultDofValue.set(0.);
      sofa::type::vector<sofa::type::Vec1d> q0Values(nqWithoutRootJoint, defaultDofValue);
      d_qRest.setValue(q0Values);
    }
    jointsDofs->resize(nqWithoutRootJoint);
    // set initial position specified from \"qInit\" data field
    jointsDofs->x.setParent(&d_qInit);
    // set rest position specified from \"qRest\" data field
    jointsDofs->x0.setParent(&d_qRest);
    robotNode->addObject(jointsDofs);

    const auto jointsNode = robotNode->createChild("Joints");

    // create mapping between robot joints dofs and its bodies placements
    const auto kinematicChainMapping = New<sofa::component::mapping::nonlinear::KinematicChainMapping<Vec1Types, Rigid3Types, Rigid3Types>>();
    kinematicChainMapping->setName("kinematicChainMapping");
    kinematicChainMapping->setBodyCoMFrames(bodyCoMFrames);
    kinematicChainMapping->m_extraFrames = extraFrames;
    kinematicChainMapping->f_printLog.setValue(true);
    kinematicChainMapping->setModel(model);

    // set mapping input1
    kinematicChainMapping->addInputModel1(jointsDofs.get());

    // set mapping output
    // one dof container for all joints
    const auto jointsDof = New<MechanicalObjectRigid3>();
    jointsDof->setName("jointsDof");
    jointsDof->resize(model->njoints + extraFrames.size());
    jointsNode->addObject(jointsDof);
    kinematicChainMapping->addOutputModel(jointsDof.get(), jointsDof->getPathName());
    // // one dof container for all extra frames
    // const auto framesDof = New<MechanicalObjectRigid3>();
    // framesDof->setName("framesDof");
    // framesDof->resize(extraFrames.size());
    // jointsNode->addObject(framesDof);
    // kinematicChainMapping->addOutputModel(framesDof.get(), framesDof->getPathName());

    jointsNode->addObject(kinematicChainMapping);

    // set mapping input2: free flyer root joint if any specified
    if(useFreeFlyerRootJoint)
    {
      const auto rootJointNode = context->createChild("RootJoint");

      const auto rootJointDof = New<MechanicalObjectRigid3>();
      rootJointDof->setName("Free-Flyer");
      rootJointDof->resize(1);
      rootJointNode->addObject(rootJointDof);

      // Bodies node have two parents: rootJointNode and robotNode
      rootJointNode->addChild(jointsNode);

      // set mapping input2
      kinematicChainMapping->addInputModel2(rootJointDof.get());
    }

    msg_info() << "-- model->nbodies" << model->nbodies;
    msg_info() << "-- model->njoints" << model->njoints;

    for (pinocchio::JointIndex jointIdx = 0; jointIdx < model->njoints; ++jointIdx)
    {
      msg_info() << "-- joint " << jointIdx;
      msg_info() << "-- joint name " << model->names[jointIdx];
      const auto jointNode = jointsNode->createChild(model->names[jointIdx]);

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
      bodyMapping->setModels(jointsDof.get(), bodyRigid.get());
      bodyMapping->d_index = jointIdx;
      bodyMapping->d_globalToLocalCoords = false;
      jointNode->addObject(bodyMapping);

      // add visual body node
      const auto visualNode = jointNode->createChild("Visual");

      // get joint associated visual geometries
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
            msg_error() << "Failed to convert pinocchio FCL visual geometry to Sofa MeshTopology";
            msg_error() << "FCL geometry object type: " << static_cast<int>(geom.geometry->getObjectType()) << ", FCL geometry node type: " << static_cast<int>(geom.geometry->getNodeType());
            continue;
          }
          visualBodyNode->addObject(visualBodyMesh);

          // add visual openGL model
          auto visualBodyModel = New<sofa::gl::component::rendering3d::OglModel>();
          // visualBodyModel->f_printLog = true;
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

      // add collision body node
      if(d_addCollision.getValue() == true)
      {
        const auto collisionNode = jointNode->createChild("Collision");
        const auto collisionData = std::make_shared<pinocchio::GeometryData>(*collisionModel);
        const auto collisionGeomIndexesIt = collisionData->innerObjects.find(jointIdx);
        if (collisionGeomIndexesIt != collisionData->innerObjects.end())
        {
          for (const auto &geomIdx : collisionGeomIndexesIt->second)
          {
            const auto &geom = collisionModel->geometryObjects[geomIdx];

            const auto collisionBodyNode = collisionNode->createChild(geom.name);

            auto collisionBodyMesh = sofa::rigidbodydynamics::fclGeometryToSofaTopology(geom.geometry, geom.placement, geom.meshScale);
            if (not collisionBodyMesh)
            {
              msg_error() << "Failed to convert pinocchio FCL collision geometry to Sofa MeshTopology";
              msg_error() << "FCL geometry object type: " << static_cast<int>(geom.geometry->getObjectType()) << ", FCL geometry node type: " << static_cast<int>(geom.geometry->getNodeType());
              continue;
            }
            collisionBodyNode->addObject(collisionBodyMesh);

            auto collisionBodyObj = New<sofa::component::statecontainer::MechanicalObject<Vec3Types>>();
            collisionBodyObj->setName("collisionMecObject");
            // collisionBodyObj->l_topology = collisionBodyMesh;
            collisionBodyNode->addObject(collisionBodyObj);

            const auto collisionMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Vec3Types>>();
            collisionMapping->setName("collisionMapping");
            collisionMapping->setModels(bodyRigid.get(), collisionBodyObj.get());
            collisionBodyNode->addObject(collisionMapping);

            // add collision model
            auto collisionTriModel = New<sofa::component::collision::geometry::TriangleCollisionModel<Vec3Types>>();
            // collisionTriModel->f_printLog.setValue(true);
            collisionTriModel->setName("collisionTriModel");
            collisionTriModel->l_topology = collisionBodyMesh;
            collisionTriModel->addGroup(1);
            collisionBodyNode->addObject(collisionTriModel);

            // auto collisionLineModel = New<sofa::component::collision::geometry::LineCollisionModel<Vec3Types>>();
            // collisionLineModel->setName("collisionLineModel");
            // collisionLineModel->l_topology = collisionBodyMesh;
            // collisionLineModel->addGroup(1);
            // collisionBodyNode->addObject(collisionLineModel);

            // auto collisionPointModel = New<sofa::component::collision::geometry::PointCollisionModel<Vec3Types>>();
            // collisionPointModel->setName("collisionPointModel");
            // // collisionPointModel->l_topology = collisionBodyMesh;
            // collisionPointModel->addGroup(1);
            // collisionBodyNode->addObject(collisionPointModel);
          }
        }
      }
    }

    msg_info() << "Model has " << model->referenceConfigurations.size() << " reference configurations registered";

    return true;
  }

} /// namespace sofa::rigidbodydynamics

int URDFModelLoaderClass = sofa::core::RegisterObject("Loads robot from URDF file.").add<sofa::rigidbodydynamics::URDFModelLoader>();