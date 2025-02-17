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
#include <sofa/RigidBodyDynamics/Types.h>

#include <SoftRobots.Inverse/component/constraint/JointActuator.h>
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
#include <pinocchio/algorithm/default-check.hpp>

using namespace sofa::defaulttype;

using MechanicalObjectVec1 = sofa::component::statecontainer::MechanicalObject<Vec1Types>;
using MechanicalObjectRigid3 = sofa::component::statecontainer::MechanicalObject<Rigid3Types>;

namespace sofa::rigidbodydynamics
{

  URDFModelLoader::URDFModelLoader() : sofa::core::loader::SceneLoader() ,
      d_modelDirectory(initData(&d_modelDirectory, "modelDirectory", "Directory containing robot models")),
      d_useFreeFlyerRootJoint(initData(&d_useFreeFlyerRootJoint, false, "useFreeFlyerRootJoint", "True if root joint is a Free Flyer joint, false if none")),
      d_addCollision(initData(&d_addCollision, true, "addCollision", "True if collision detection must be enabled for the robot (self-collision and other objects)")),
      d_addJointsActuators(initData(&d_addJointsActuators, false, "addJointsActuators", "True if SoftRobots.Inverse actuators objects must be set for each robot joint")),
      d_qRest(initData(&d_qRest, "qRest", "Rest configuration values of robot DoFs")),
      d_qInit(initData(&d_qInit, "qInit", "Initial configuration values of robot DoFs"))
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
      const auto modelNode = context->getChild("Model");
      if (modelNode)
      {
        context->removeChild(modelNode);
        msg_info() << "Model node was already present in robot scene tree. Removing it...";
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

      if(not model->check())
      {
        msg_warning() << "Model failed checking. URDF model does not comply with pinocchio specifications";
      }
      else
      {
        msg_info() << "Model checked successfully";
      }
      msg_info() << "Robot nq = " << model->nq << " / nv = " << model->nv;
      msg_info() << "Robot njoints = " << model->njoints << " (incl. \"universe\") / nbodies = " << model->nbodies << " / nframes = " << model->nframes;
      msg_info() << "Robot model 6d gravity g = " << model->gravity;
      msg_info() << "Robot inertias vector size = " << model->inertias.size();

      for(pinocchio::JointIndex jointIdx = 0u; jointIdx < model->njoints; ++jointIdx)
      {
        msg_info() << "Joint[" << jointIdx << "] (index = " << model->joints[jointIdx].id() << "): " << model->names[jointIdx] << " / " << model->joints[jointIdx];
      }

      // TODO should be configurable to ligthen computations
      msg_info() << "Adding all frames (num = " << model->nframes << ") to kinematic mapping...";
      for(auto frameIdx = 0u; frameIdx < model->nframes; ++frameIdx)
      {
        msg_info() << "Frame[" << frameIdx << "]: " << model->frames[frameIdx].name <<" / parent Joint = " << model->frames[frameIdx].parentJoint << " / parent Frame = " << model->frames[frameIdx].parentFrame;
        extraFrames.push_back(frameIdx);
      }

      // TODO use collisionModel to create collision nodes in SOFA
      collisionModel = std::make_shared<pinocchio::GeometryModel>();
      pinocchio::urdf::buildGeom(*model, urdfFilename, pinocchio::COLLISION, *collisionModel, modelDir);
      // msg_info() << "Built robot collision model from URDF file: " << urdf_filename;

      visualModel = std::make_shared<pinocchio::GeometryModel>();
      pinocchio::urdf::buildGeom(*model, urdfFilename, pinocchio::VISUAL, *visualModel, modelDir);
      // msg_info() << "Built robot visual model from URDF file: " << urdf_filename;

      // add a frame for each body centered on its CoM and that will be used as body DoF by SOFA
      for (pinocchio::JointIndex jointIdx = kSkipUniverse; jointIdx < model->njoints; ++jointIdx)
      {
        const pinocchio::SE3 bodyCoM_i = pinocchio::SE3(Eigen::Matrix3d::Identity(), model->inertias[jointIdx].lever());
        const auto bodyFrameCoM = pinocchio::Frame{"Body_" + std::to_string(jointIdx) + "_CoM", jointIdx, bodyCoM_i, pinocchio::FrameType::OP_FRAME};
        bodyCoMFrames.push_back(model->addFrame(bodyFrameCoM));
        msg_info() << "==== bodyCoMFrames[" << jointIdx << "]: " << bodyCoMFrames.back();
      }
    }
    catch (std::exception &e)
    {
      msg_error() << "Caught exception: " << e.what();
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return false;
    }

    const int num_joints = model->njoints - kSkipUniverse; // if we do not want universe joint

    // create robot scene tree
    const auto robotNode = context->createChild("Robot");

    const auto jointsDofs = New<MechanicalObjectVec1>();
    jointsDofs->setName("dofs");
    auto nqWithoutRootJoint = useFreeFlyerRootJoint ? model->nq - 7 : model->nq;
    msg_info() << "nqWithoutRootJoint = " << nqWithoutRootJoint;

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


    // if(d_addJointsActuators.getValue() == true)
    // {
    //   for(JointIndex jointIdx = 0; jointIdx < model->njoints - 1; ++jointIdx)
    //   {
    //     // revolute joint case
    //     const auto& joint = model->joints[jointIdx + 1];
    //     msg_info() << "** joint[" << jointIdx + 1 << "]: " << joint.classname() << " / shortname: " << joint.shortname();
    //     if(joint.shortname().rfind("JointModelR", 0) == 0)
    //     {
    //       // const int jointConfigIndex = joint.idx_q(); // here we know that this type of joint is only described by one parameter
    //       // msg_info() << "**** joint[" << jointIdx + 1 << "]: jointConfigIndex" << jointConfigIndex;
    //       // if(jointConfigIndex < 0)
    //       // {
    //       //   continue;
    //       // }
    //       const auto jointActuator = New<softrobotsinverse::constraint::JointActuator<sofa::defaulttype::Vec1Types>>();
    //       jointActuator->setName("actuator_" + model->names[jointIdx + 1]);
    //       jointActuator->d_index = jointIdx;
    //       // jointActuator->d_minAngle = model->lowerPositionLimit[jointConfigIndex];
    //       // jointActuator->d_maxAngle = model->upperPositionLimit[jointConfigIndex];
    //       jointActuator->d_minAngle = -3.14;
    //       jointActuator->d_maxAngle = 3.14;
    //       jointActuator->d_maxAngleVariation = 0.005; // TODO
    //       robotNode->addObject(jointActuator);
    //       msg_info() << "****** added joint actuator for joint[" << jointIdx + 1 << "] (name = "<< model->names[jointIdx + 1] << " with index = " << jointActuator->d_index;
    //     }
    //     else
    //     {
    //       msg_error() << "Unsupported type of joint actuator: " << joint.shortname();
    //       d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
    //       return false;
    //     }
    //   }
    // }

    const auto modelNode = robotNode->createChild("Model");

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
    // one dof container for all joints and extra frames
    const auto mappedDof = New<MechanicalObjectRigid3>();
    mappedDof->setName("mappedDof");
    mappedDof->resize(num_joints + extraFrames.size());
    modelNode->addObject(mappedDof);

    kinematicChainMapping->addOutputModel(mappedDof.get());

    modelNode->addObject(kinematicChainMapping);

    // set joints
    const auto jointsNode = robotNode->createChild("Joints");
    const auto jointsDof = New<MechanicalObjectRigid3>();
    jointsDof->setName("jointsDof");
    jointsDof->resize(num_joints);
    jointsNode->addObject(jointsDof);
    const auto jointsMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Rigid3Types>>();
    jointsMapping->setName("jointsMapping");
    jointsMapping->setModels(mappedDof.get(), jointsDof.get());
    std::vector<unsigned int> jointsDofIndexes;
    for(JointIndex i = 0; i < num_joints; ++i)
    {
      jointsDofIndexes.push_back(i);
    }
    jointsMapping->d_rigidIndexPerPoint = jointsDofIndexes;
    jointsMapping->d_globalToLocalCoords = false;
    jointsNode->addObject(jointsMapping);

    // set frames
    const auto framesNode = robotNode->createChild("Frames");
    const auto framesDof = New<MechanicalObjectRigid3>();
    framesDof->setName("framesDof");
    framesDof->resize(extraFrames.size());
    framesNode->addObject(framesDof);
    const auto framesMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Rigid3Types>>();
    framesMapping->setName("framesMapping");
    framesMapping->setModels(mappedDof.get(), framesDof.get());
    std::vector<unsigned int> framesDofIndexes;
    for(auto i = 0; i < extraFrames.size(); ++i)
    {
      framesDofIndexes.push_back(num_joints + i);
      // create a node + submapping for each frame
      const auto& frame = model->frames[extraFrames[i]];
      const auto frameNode = framesNode->createChild(frame.name);
      const auto frameDof = New<MechanicalObjectRigid3>();
      frameDof->setName("dof");
      frameNode->addObject(frameDof);
      const auto frameMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Rigid3Types>>();
      frameMapping->setName("frameMapping");
      frameMapping->setModels(framesDof.get(), frameDof.get());  
      frameMapping->d_index = i;
      frameNode->addObject(frameMapping);
    }
    framesMapping->d_rigidIndexPerPoint = framesDofIndexes;
    framesMapping->d_globalToLocalCoords = false;
    framesNode->addObject(framesMapping);

    // set mapping input2: free flyer root joint if any specified
    if(useFreeFlyerRootJoint)
    {
      const auto rootJointNode = context->createChild("RootJoint");

      const auto rootJointDof = New<MechanicalObjectRigid3>();
      rootJointDof->setName("Free-Flyer");
      rootJointDof->resize(1);
      rootJointNode->addObject(rootJointDof);

      // Joints node have two parents: rootJointNode and robotNode
      rootJointNode->addChild(jointsNode);

      // set mapping input2
      kinematicChainMapping->addInputModel2(rootJointDof.get());
    }

    msg_info() << "-- pinocchio model num bodies: " << model->nbodies;
    msg_info() << "-- pinocchio model num joints: " << model->njoints;
    msg_info() << "-- SOFA converted model num_joints : " << num_joints;

    for (JointIndex jointIdx = 0; jointIdx < num_joints; ++jointIdx)
    {
      const pinocchio::JointIndex pinoJointIdx = jointIdx + kSkipUniverse;

      msg_info() << "-- pinocchio joint idx: " << pinoJointIdx;
      msg_info() << "-- joint name " << model->names[pinoJointIdx];
      msg_info() << "-- joint shortname " << model->joints[pinoJointIdx].shortname();

      const auto jointNode = jointsNode->createChild(model->names[pinoJointIdx]);
      const auto& jointInertia = model->inertias[pinoJointIdx];

      const auto bodyRigid = New<MechanicalObjectRigid3>();
      bodyRigid->setName("jointRigid");
      const Eigen::Vector3d invBodyCoMTranslation = -jointInertia.lever();
      bodyRigid->setTranslation(invBodyCoMTranslation.x(), invBodyCoMTranslation.y(), invBodyCoMTranslation.z());
      jointNode->addObject(bodyRigid);

      const auto bodyMass = New<sofa::component::mass::UniformMass<Rigid3Types>>();
      bodyMass->setName("mass");
      sofa::defaulttype::Rigid3dMass rigidMass;
      rigidMass.mass = jointInertia.mass();
      const Eigen::Matrix3d massInertia = jointInertia.inertia().matrix();
      const Eigen::Matrix3d inertiaDivByMass = massInertia / jointInertia.mass();

      rigidMass.inertiaMatrix = sofa::rigidbodydynamics::mat3ToSofaType(inertiaDivByMass);
      rigidMass.volume = 1.; // XXX: should not be used here as we only deal with rigid bodies, so we should be able to set any value

      msg_info() << "-- inertia matrix (sofa) = " << rigidMass.inertiaMatrix;
      msg_info() << "----- det = " << determinant(rigidMass.inertiaMatrix);

      rigidMass.recalc();
      bodyMass->setMass(rigidMass);
      jointNode->addObject(bodyMass);

      const auto bodyMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Rigid3Types>>();
      bodyMapping->setName("jointMapping");
      bodyMapping->setModels(mappedDof.get(), bodyRigid.get());
      bodyMapping->d_index = jointIdx;
      bodyMapping->d_globalToLocalCoords = false;
      jointNode->addObject(bodyMapping);

      // add visual body node
      const auto visualNode = jointNode->createChild("Visual");

      // get joint associated visual geometries
      const auto visualData = std::make_shared<pinocchio::GeometryData>(*visualModel);
      const auto visualGeomIndexesIt = visualData->innerObjects.find(pinoJointIdx);
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
            d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
            return false;
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
        const auto collisionGeomIndexesIt = collisionData->innerObjects.find(pinoJointIdx);
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
              d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
              return false;
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