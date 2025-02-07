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

#include <sofa/RigidBodyDynamics/GeometryConversions.h>

#include <sofa/RigidBodyDynamics/BaseShapesConversions.h>

#include <coal/BVH/BVH_model.h>
#include <coal/shape/geometric_shapes.h>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>

namespace
{
  static const int kDefaultCylinderResolution = 8;

  static const int kDefaultSphereNumStacks = 4;
  static const int kDefaultSphereNumSlices = 8;

  sofa::component::topology::container::constant::MeshTopology::SPtr
  fclBaseShapeGeometryToSofaTopology(const std::shared_ptr<coal::CollisionGeometry> &geom,
    const pinocchio::SE3& tf,
    const Eigen::Vector3d& scale)
  {
    sofa::component::topology::container::constant::MeshTopology::SPtr meshTopology = nullptr;

    // we do not need dynamic cast safe checking here as type is ensured by the switch/case on node type,
    // but as we do not care about high performances here and prefer safer code
    switch (geom->getNodeType())
    {
    case coal::GEOM_BOX:
    {
      const auto boxGeom = std::dynamic_pointer_cast<coal::Box>(geom);
      assert(boxGeom);

      meshTopology = sofa::rigidbodydynamics::boxToMeshTopology(boxGeom, tf, scale);

      break;
    }
    case coal::GEOM_CYLINDER:
    {
      const auto cylGeom = std::dynamic_pointer_cast<coal::Cylinder>(geom);
      assert(cylGeom);

      meshTopology = sofa::rigidbodydynamics::cylinderToMeshTopology(cylGeom, tf, scale, kDefaultCylinderResolution);

      break;
    }
    case coal::GEOM_SPHERE:
    {
      const auto sphereGeom = std::dynamic_pointer_cast<coal::Sphere>(geom);
      assert(sphereGeom);

      meshTopology = sofa::rigidbodydynamics::sphereToMeshTopology(sphereGeom, tf, scale, kDefaultSphereNumStacks, kDefaultSphereNumSlices);

      break;
    }

    default:
      // unsupported fcl geometry node type, return nullptr
      break;
    }

    return meshTopology;
  }

} // anonymous namespace

namespace sofa::rigidbodydynamics
{
  sofa::component::topology::container::constant::MeshTopology::SPtr
  fclGeometryToSofaTopology(const std::shared_ptr<coal::CollisionGeometry> &geom,
    const pinocchio::SE3& tf,
    const Eigen::Vector3d& scale)
  {
    sofa::component::topology::container::constant::MeshTopology::SPtr meshTopology = nullptr;

    // we do not need dynamic cast safe checking here as type is ensured by the switch/case on object type,
    // but as we do not care about high performances here and prefer safer code
    switch (geom->getObjectType())
    {
    case coal::OT_BVH:
    {
      const auto bvhGeom = std::dynamic_pointer_cast<coal::BVHModelBase>(geom);
      assert(bvhGeom);

      meshTopology = sofa::core::objectmodel::New<sofa::component::topology::container::constant::MeshTopology>();
      meshTopology->setName("mesh");
      if (bvhGeom->getModelType() == coal::BVH_MODEL_TRIANGLES)
      {
        // reconstruct mesh
        for (auto vertIdx = 0ul; vertIdx < bvhGeom->num_vertices; ++vertIdx)
        {
          const pinocchio::SE3::Vector3 fclVert = tf.act((*bvhGeom->vertices)[vertIdx]).cwiseProduct(scale.cwiseAbs());
          meshTopology->addPoint(fclVert[0], fclVert[1], fclVert[2]);

        }
        for (auto triIdx = 0ul; triIdx < bvhGeom->num_tris; ++triIdx)
        {
          const auto& fclTri = (*bvhGeom->tri_indices)[triIdx];
          meshTopology->addTriangle(fclTri[0], fclTri[1], fclTri[2]);
        }
      }
      break;
    }
    case coal::OT_GEOM:
      meshTopology = fclBaseShapeGeometryToSofaTopology(geom, tf, scale);
      break;
    default:
      // uunsupported fcl geometry object type, return nullptr
      return meshTopology;
      break;
    }

    return meshTopology;
  }

} // namespace sofa::rigidbodydynamics
