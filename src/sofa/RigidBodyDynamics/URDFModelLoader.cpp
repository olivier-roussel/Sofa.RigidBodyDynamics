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
// #include <sofa/component/topology/container/constant/MeshTopology.h>
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
      : d_urdfFilename(initData(&d_urdfFilename, "urdfFilename", "Filename of the URDF model")), d_modelDirectory(initData(&d_modelDirectory, "modelDirectory", "Directory containing robot models"))
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

  void URDFModelLoader::init()
  {
    this->reinit();
  }

  void URDFModelLoader::reinit()
  {
    const std::string &urdfFilename = d_urdfFilename.getValue();
    const std::string &modelDir = d_modelDirectory.getValue();
    msg_info() << " Loading robot from URDF file: " << urdfFilename;
    msg_info() << "Model directory: " << modelDir;
    std::shared_ptr<pinocchio::Model> model;
    std::shared_ptr<pinocchio::GeometryModel> collisionModel;
    std::shared_ptr<pinocchio::GeometryModel> visualModel;

    try
    {
      model = std::make_shared<pinocchio::Model>();
      pinocchio::urdf::buildModel(urdfFilename, *model);
      msg_info() << "Built robot model from URDF file: " << urdfFilename;
      msg_info() << "Robot nq = " << model->nq << " / nv = " << model->nv;
      msg_info() << "Robot njoints = " << model->njoints << " / nbodies = " << model->nbodies << " / nframes = " << model->nframes;
      msg_info() << "Robot model 6d gravity g = " << model->gravity;

      collisionModel = std::make_shared<pinocchio::GeometryModel>();
      pinocchio::urdf::buildGeom(*model, urdfFilename, pinocchio::COLLISION, *collisionModel, modelDir);
      // msg_info() << "Built robot collision model from URDF file: " << urdf_filename;

      visualModel = std::make_shared<pinocchio::GeometryModel>();
      pinocchio::urdf::buildGeom(*model, urdfFilename, pinocchio::VISUAL, *visualModel, modelDir);
      // msg_info() << "Built robot visual model from URDF file: " << urdf_filename;
    }
    catch (std::exception &e)
    {
      msg_error() << "Caught exception: " << e.what();
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }

    // create robot scene tree
    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
    {
      auto jointsNode = context->getChild("Joints");
      if (jointsNode)
      {
        context->removeChild(jointsNode);
        msg_info() << "Joints node was already present in robot scene tree. Removing it...";
      }
      auto bodiesNode = context->getChild("Bodies");
      if (bodiesNode)
      {
        context->removeChild(bodiesNode);
        msg_info() << "Bodies node was already present in robot scene tree. Removing it...";
      }
    }
    const auto jointsNode = context->createChild("Joints");
    const auto jointsDofs = New<MechanicalObjectVec1>();
    jointsDofs->setName("dofs");
    jointsDofs->resize(model->nq);
    // set desired position specified from \"q0\" data field
    auto robotJointsData = context->findData("q0");
    if (not robotJointsData)
    {
      msg_error() << "joints data \"q0\" not found ";
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return;
    }
    jointsDofs->x0.setParent(robotJointsData);

    jointsNode->addObject(jointsDofs);

    // TODO -> set actual mass per body and other inertia properties
    const auto jointsMass = New<sofa::component::mass::UniformMass<Vec1Types>>();
    jointsMass->setName("mass");
    jointsMass->setTotalMass(1.);
    jointsNode->addObject(jointsMass);

    // spring to control robot dofs to desired dofs set by rest position (x0)
    const auto restShapeForceField = New<sofa::component::solidmechanics::spring::RestShapeSpringsForceField<Vec1Types>>();
    restShapeForceField->setName("RestShapeSpringsForceField");
    std::vector<double> springVal = {1e10};
    restShapeForceField->d_stiffness.setValue(springVal);
    sofa::type::vector<sofa::Index> pointsIndexes;
    for (sofa::Index idx = 0ul; idx < model->nq; ++idx)
    {
      pointsIndexes.push_back(idx);
    }
    restShapeForceField->d_points.setValue(pointsIndexes);
    jointsNode->addObject(restShapeForceField);

    const auto bodiesNode = jointsNode->createChild("Bodies");

    // create mapping between robot joints dofs and its bodies placements
    const auto kinematicChainMapping = New<sofa::component::mapping::KinematicChainMapping<Vec1Types, Rigid3Types, Rigid3Types>>();
    kinematicChainMapping->setName("kinematicChainMapping");
    kinematicChainMapping->f_printLog.setValue(true);
    kinematicChainMapping->setModel(model);
    kinematicChainMapping->setCollisionModel(collisionModel);
    kinematicChainMapping->setVisualModel(visualModel);
    // set mapping input1
    kinematicChainMapping->addInputModel1(jointsDofs.get());
    // TODO set mapping input2 (free flyer base dof if any)

    // one dof container for all bodies version
    const auto bodiesDof = New<MechanicalObjectRigid3>();
    bodiesDof->setName("bodiesDofs");
    bodiesDof->resize(model->njoints);
    bodiesNode->addObject(bodiesDof);

    kinematicChainMapping->addOutputModel(bodiesDof.get());
    bodiesNode->addObject(kinematicChainMapping);

    for (pinocchio::JointIndex jointIdx = 0; jointIdx < model->njoints; ++jointIdx)
    {
      const auto bodyNode = bodiesNode->createChild("Body_" + std::to_string(jointIdx));

      const auto bodyRigid = New<MechanicalObjectRigid3>();
      bodyRigid->setName("bodyRigid");
      bodyNode->addObject(bodyRigid);

      const auto bodyMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Rigid3Types>>();
      bodyMapping->setName("bodyMapping");
      bodyMapping->setModels(bodiesDof.get(), bodyRigid.get());
      bodyMapping->d_index = jointIdx;
      bodyMapping->d_globalToLocalCoords = false;
      bodyNode->addObject(bodyMapping);

      // add visual body node
      const auto visualNode = bodyNode->createChild("Visual");

      // get joint associated geometries
      const auto visualGeomIndexesIt = kinematicChainMapping->visualData()->innerObjects.find(jointIdx);
      if (visualGeomIndexesIt != kinematicChainMapping->visualData()->innerObjects.end())
      {
        for (const auto &geomIdx : visualGeomIndexesIt->second)
        {
          const auto &geom = visualModel->geometryObjects[geomIdx];

          const auto visualBodyNode = visualNode->createChild(geom.name);
          msg_info() << "joint[" << jointIdx << "]:geom name: " << geom.name << " / parent joint: " << geom.parentJoint << " / object type: " << static_cast<int>(geom.fcl->getObjectType()) << " / node type: " << static_cast<int>(geom.fcl->getNodeType());

          auto visualBodyMesh = sofa::rigidbodydynamics::fclGeometryToSofaTopology(geom.geometry, geom.placement, geom.meshScale);
          if (not visualBodyMesh)
          {
            msg_error() << "Failed to convert pinocchio FCL geometry to Sofa MeshTopology";
            msg_error() << "FCL geometry object type: " << static_cast<int>(geom.geometry->getObjectType()) << ", FCL geometry node type: " << static_cast<int>(geom.geometry->getNodeType());
            d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
            return;
          }
          visualBodyNode->addObject(visualBodyMesh);

          // add visual openGL model (should be linked automatically to the topology)
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