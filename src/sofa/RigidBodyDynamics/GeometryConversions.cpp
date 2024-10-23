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
#include <hpp/fcl/shape/geometric_shapes.h>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>

namespace
{

  sofa::component::topology::container::constant::MeshTopology::SPtr
  fclBaseShapeGeometryToSofaTopology(const std::shared_ptr<hpp::fcl::CollisionGeometry> &geom, 
    const pinocchio::SE3& tf,
    const Eigen::Vector3d& scale)
  {
    sofa::component::topology::container::constant::MeshTopology::SPtr meshTopology = nullptr;

    // we do not need dynamic cast safe checking here as type is ensured by the switch/case on node type,
    // but as we do not care about high performances here and prefer safer code
    switch (geom->getNodeType())
    {
    case hpp::fcl::GEOM_BOX:
    {
      const auto boxGeom = std::dynamic_pointer_cast<hpp::fcl::Box>(geom);
      assert(boxGeom);
      meshTopology = sofa::core::objectmodel::New<sofa::component::topology::container::constant::MeshTopology>();
      meshTopology->setName("box");

      // add box vertices
      auto boxVerts = std::vector<Eigen::Vector3d>();
      boxVerts.emplace_back(+boxGeom->halfSide[0], -boxGeom->halfSide[1], -boxGeom->halfSide[2]); // 0
      boxVerts.emplace_back(+boxGeom->halfSide[0], -boxGeom->halfSide[1], +boxGeom->halfSide[2]); // 1
      boxVerts.emplace_back(-boxGeom->halfSide[0], -boxGeom->halfSide[1], +boxGeom->halfSide[2]); // 2
      boxVerts.emplace_back(-boxGeom->halfSide[0], -boxGeom->halfSide[1], -boxGeom->halfSide[2]); // 3
      boxVerts.emplace_back(+boxGeom->halfSide[0], +boxGeom->halfSide[1], -boxGeom->halfSide[2]); // 4
      boxVerts.emplace_back(-boxGeom->halfSide[0], +boxGeom->halfSide[1], -boxGeom->halfSide[2]); // 5
      boxVerts.emplace_back(-boxGeom->halfSide[0], +boxGeom->halfSide[1], +boxGeom->halfSide[2]); // 6
      boxVerts.emplace_back(+boxGeom->halfSide[0], +boxGeom->halfSide[1], +boxGeom->halfSide[2]); // 7

      // apply transfrom and set to mesh data
      for(const auto& pt: boxVerts)
      {
        const pinocchio::SE3::Vector3 ptTf = tf.act(pt).cwiseProduct(scale.cwiseAbs());
        meshTopology->addPoint(ptTf[0], ptTf[1], ptTf[2]);
      }
      // add faces
      meshTopology->addQuad(0, 1, 2, 3);
      meshTopology->addQuad(4, 5, 6, 7);
      meshTopology->addQuad(4, 7, 1, 0);
      meshTopology->addQuad(3, 2, 6, 5);
      meshTopology->addQuad(1, 7, 6, 2);
      meshTopology->addQuad(0, 3, 5, 4);

      break;
    }
    case hpp::fcl::GEOM_CYLINDER:
    {
      const auto cylGeom = std::dynamic_pointer_cast<hpp::fcl::Cylinder>(geom);
      assert(cylGeom);
      meshTopology = sofa::core::objectmodel::New<sofa::component::topology::container::constant::MeshTopology>();
      meshTopology->setName("cylinder");

      static const unsigned int kCylinderResolution = 8;
      static const double kPi = 3.141592653;

      for (auto i = 0; i < kCylinderResolution; ++i)
      {
        const double theta = (i / static_cast<double>(kCylinderResolution)) * (2 * kPi);
        const auto x = std::cos(theta) * cylGeom->radius;
        const auto y = std::sin(theta) * cylGeom->radius;
        const Eigen::Vector3d bottom = Eigen::Vector3d{x, y, -cylGeom->halfLength};
        const Eigen::Vector3d top = Eigen::Vector3d{x, y, cylGeom->halfLength};
        const pinocchio::SE3::Vector3 bottomTf = tf.act(bottom).cwiseProduct(scale.cwiseAbs());
        meshTopology->addPoint(bottomTf[0], bottomTf[1], bottomTf[2]);
        const pinocchio::SE3::Vector3 topTf = tf.act(top).cwiseProduct(scale.cwiseAbs());
        meshTopology->addPoint(topTf[0], topTf[1], topTf[2]);

        // add lateral face quad
        const auto ii = i * 2;
        const auto jj = (ii + 2) % (kCylinderResolution * 2);
        const auto kk = (ii + 3) % (kCylinderResolution * 2);
        const auto ll = ii + 1;
        meshTopology->addQuad(ii, jj, kk, ll);
      }
      for (auto i = 0; i < kCylinderResolution - 2; ++i)
      {
        // add top face
        meshTopology->addTriangle(1, 2*(i+1)+1, 2*(i+2)+1);
        // add bottom face
        meshTopology->addTriangle(0, 2*(i+2), 2*(i+1));
      }

    }
    default:
      // unsupported fcl geometry node type, return nullptr
      return meshTopology;
      break;
    }

    return meshTopology;
  }

} // anonymous namespace

namespace sofa::rigidbodydynamics
{
  sofa::component::topology::container::constant::MeshTopology::SPtr
  fclGeometryToSofaTopology(const std::shared_ptr<hpp::fcl::CollisionGeometry> &geom, 
    const pinocchio::SE3& tf,
    const Eigen::Vector3d& scale)
  {
    sofa::component::topology::container::constant::MeshTopology::SPtr meshTopology = nullptr;

    // we do not need dynamic cast safe checking here as type is ensured by the switch/case on object type,
    // but as we do not care about high performances here and prefer safer code
    switch (geom->getObjectType())
    {
    case hpp::fcl::OT_BVH:
    {
      const auto bvhGeom = std::dynamic_pointer_cast<hpp::fcl::BVHModelBase>(geom);
      assert(bvhGeom);
      meshTopology = sofa::core::objectmodel::New<sofa::component::topology::container::constant::MeshTopology>();
      meshTopology->setName("mesh");
      if (bvhGeom->getModelType() == hpp::fcl::BVH_MODEL_TRIANGLES)
      {
        // reconstruct mesh
        for (auto vertIdx = 0ul; vertIdx < bvhGeom->num_vertices; ++vertIdx)
        {
          const pinocchio::SE3::Vector3 fclVert = tf.act(bvhGeom->vertices[vertIdx]).cwiseProduct(scale.cwiseAbs());
          meshTopology->addPoint(fclVert[0], fclVert[1], fclVert[2]);

        }
        for (auto triIdx = 0ul; triIdx < bvhGeom->num_tris; ++triIdx)
        {
          const auto& fclTri = bvhGeom->tri_indices[triIdx];
          meshTopology->addTriangle(fclTri[0], fclTri[1], fclTri[2]);
        }
      }
      break;
    }
    case hpp::fcl::OT_GEOM:
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
