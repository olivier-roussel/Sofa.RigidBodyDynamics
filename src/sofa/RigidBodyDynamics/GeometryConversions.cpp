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

#include <hpp/fcl/BVH/BVH_model.h>

namespace sofa::rigidbodydynamics
{
  sofa::component::topology::container::constant::MeshTopology::SPtr
  fclGeometryToSofaTopology(const std::shared_ptr<hpp::fcl::CollisionGeometry> &geom)
  {
    const auto visualBodyMesh = sofa::core::objectmodel::New<sofa::component::topology::container::constant::MeshTopology>();
    visualBodyMesh->setName("mesh");

    switch (geom->getObjectType())
    {
    case hpp::fcl::OT_BVH:
    {
      // we do not need dynamic cast safe checking here as type is ensured by the switch/case on object type,
      // but as we do not care about high performances here we prefer safer code
      auto bvh_geom = std::dynamic_pointer_cast<hpp::fcl::BVHModelBase>(geom);
      assert(bvh_geom);
      if (bvh_geom->getModelType() == hpp::fcl::BVH_MODEL_TRIANGLES)
      {
        // reconstruct mesh
        Eigen::Vector3d scale = Eigen::Vector3d::Ones(); // TODO use geom.meshScale instead
        for (auto vertIdx = 0ul; vertIdx < bvh_geom->num_vertices; ++vertIdx)
        {
          auto fclVert = bvh_geom->vertices[vertIdx];
          visualBodyMesh->addPoint(fclVert[0] * scale[0], fclVert[1] * scale[1], fclVert[2] * scale[2]);
        }
        for (auto triIdx = 0ul; triIdx < bvh_geom->num_tris; ++triIdx)
        {
          auto fclTri = bvh_geom->tri_indices[triIdx];
          visualBodyMesh->addTriangle(fclTri[0], fclTri[1], fclTri[2]);
        }
      }
    }
    break;

    default:
      // uunsupported fcl geometry type
      return nullptr;
      break;
    }

    return visualBodyMesh;
  }

} // namespace sofa::rigidbodydynamics
