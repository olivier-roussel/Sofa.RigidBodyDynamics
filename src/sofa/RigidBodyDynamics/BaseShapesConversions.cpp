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
#include <sofa/RigidBodyDynamics/BaseShapesConversions.h>

namespace
{
  static const double kPi = 3.1415926535897932384626433832795029;

  inline
  void
  addTransformedPointToMesh(const Eigen::Vector3d& point,
    const sofa::component::topology::container::constant::MeshTopology::SPtr& mesh, 
    const pinocchio::SE3& tf,
    const Eigen::Vector3d& scale)
  {
    const pinocchio::SE3::Vector3 ptTf = tf.act(point).cwiseProduct(scale.cwiseAbs());
    mesh->addPoint(ptTf[0], ptTf[1], ptTf[2]);
  }

}

namespace sofa::rigidbodydynamics
{
  sofa::component::topology::container::constant::MeshTopology::SPtr
  boxToMeshTopology(const std::shared_ptr<coal::Box>& geom,
    const pinocchio::SE3& tf,
    const Eigen::Vector3d& scale)
    {
      auto meshTopology = sofa::core::objectmodel::New<sofa::component::topology::container::constant::MeshTopology>();
      meshTopology->setName("box");

      // add box vertices
      auto boxVerts = std::vector<Eigen::Vector3d>();
      boxVerts.emplace_back(+geom->halfSide[0], -geom->halfSide[1], -geom->halfSide[2]); // 0
      boxVerts.emplace_back(+geom->halfSide[0], -geom->halfSide[1], +geom->halfSide[2]); // 1
      boxVerts.emplace_back(-geom->halfSide[0], -geom->halfSide[1], +geom->halfSide[2]); // 2
      boxVerts.emplace_back(-geom->halfSide[0], -geom->halfSide[1], -geom->halfSide[2]); // 3
      boxVerts.emplace_back(+geom->halfSide[0], +geom->halfSide[1], -geom->halfSide[2]); // 4
      boxVerts.emplace_back(-geom->halfSide[0], +geom->halfSide[1], -geom->halfSide[2]); // 5
      boxVerts.emplace_back(-geom->halfSide[0], +geom->halfSide[1], +geom->halfSide[2]); // 6
      boxVerts.emplace_back(+geom->halfSide[0], +geom->halfSide[1], +geom->halfSide[2]); // 7

      // apply transfrom and set to mesh data
      for(const auto& pt: boxVerts)
      {
        addTransformedPointToMesh(pt, meshTopology, tf, scale);
      }
      // add faces
      meshTopology->addQuad(0, 1, 2, 3);
      meshTopology->addQuad(4, 5, 6, 7);
      meshTopology->addQuad(4, 7, 1, 0);
      meshTopology->addQuad(3, 2, 6, 5);
      meshTopology->addQuad(1, 7, 6, 2);
      meshTopology->addQuad(0, 3, 5, 4);

      return meshTopology;
    }

  sofa::component::topology::container::constant::MeshTopology::SPtr
  cylinderToMeshTopology(const std::shared_ptr<coal::Cylinder>& geom,
    const pinocchio::SE3& tf,
    const Eigen::Vector3d& scale,
    unsigned int resolution)
    {
      assert(resolution > 2);

      auto meshTopology = sofa::core::objectmodel::New<sofa::component::topology::container::constant::MeshTopology>();
      meshTopology->setName("cylinder");

      for (auto i = 0; i < resolution; ++i)
      {
        const double theta = (i / static_cast<double>(resolution)) * (2 * kPi);
        const auto x = std::cos(theta) * geom->radius;
        const auto y = std::sin(theta) * geom->radius;
        const Eigen::Vector3d bottom = Eigen::Vector3d{x, y, -geom->halfLength};
        const Eigen::Vector3d top = Eigen::Vector3d{x, y, geom->halfLength};
        addTransformedPointToMesh(bottom, meshTopology, tf, scale);
        addTransformedPointToMesh(top, meshTopology, tf, scale);

        // add lateral face quad
        const auto ii = i * 2;
        const auto jj = (ii + 2) % (resolution * 2);
        const auto kk = (ii + 3) % (resolution * 2);
        const auto ll = ii + 1;
        meshTopology->addQuad(ii, jj, kk, ll);
      }
      for (auto i = 0; i < resolution - 2; ++i)
      {
        // add top face
        meshTopology->addTriangle(1, 2*(i+1)+1, 2*(i+2)+1);
        // add bottom face
        meshTopology->addTriangle(0, 2*(i+2), 2*(i+1));
      }

      return meshTopology;
    }

  sofa::component::topology::container::constant::MeshTopology::SPtr
  sphereToMeshTopology(const std::shared_ptr<coal::Sphere>& geom,
    const pinocchio::SE3& tf,
    const Eigen::Vector3d& scale,
    unsigned int numStacks,
    unsigned int numSlices)
    {
      assert(numStacks > 2);
      assert(numSlices > 2);

      auto meshTopology = sofa::core::objectmodel::New<sofa::component::topology::container::constant::MeshTopology>();
      meshTopology->setName("sphere");

      const Eigen::Vector3d top = Eigen::Vector3d{0., 0., geom->radius};
      addTransformedPointToMesh(top, meshTopology, tf, scale);

      for (auto i = 0; i < numStacks - 1; ++i)
      {
        const auto theta = kPi * (i + 1) / static_cast<double>(numStacks);
        for (auto j = 0; j < numSlices; ++j)
        {
          const double phi = (j / static_cast<double>(numSlices)) * (2 * kPi);
          const auto x = std::sin(theta) * std::cos(phi) * geom->radius;
          const auto y = std::sin(theta) * std::sin(phi) * geom->radius;
          const auto z = std::cos(theta) * geom->radius;
          const Eigen::Vector3d pt = Eigen::Vector3d{x, y, z};
          addTransformedPointToMesh(pt, meshTopology, tf, scale);
        }
      }
      const Eigen::Vector3d bottom = Eigen::Vector3d{0., 0., -geom->radius};
      addTransformedPointToMesh(bottom, meshTopology, tf, scale);
      const auto numVerts = (numStacks - 1) * numSlices + 2;
      assert(numVerts == meshTopology->getNbPoints());

      // add triangles for first (z+) / last (z-) stack
      for (auto i = 0; i < numSlices; ++i)
      {
        const auto i1p = i + 1;
        const auto i2p = (i + 1) % numSlices + 1;
        meshTopology->addTriangle(0, i1p, i2p);
        const auto i1n = i + numSlices * (numStacks - 2) + 1;
        const auto i2n = (i + 1) % numSlices + numSlices * (numStacks - 2) + 1;
        meshTopology->addTriangle(numVerts - 1, i2n, i1n);
      }

      // add quads for each sliced stack
      for(int i = 0; i < numStacks - 2; ++i)
      {
        const auto i0 = i * numSlices + 1;
        const auto i1 = (i + 1) * numSlices + 1;

        for(int j = 0; j < numSlices; ++j)
        {
          const auto j0 = i0 + j;
          const auto j1 = i0 + (j + 1) % numSlices;
          const auto j2 = i1 + (j + 1) % numSlices;
          const auto j3 = i1 + j;
          meshTopology->addQuad(j0, j3, j2, j1);
        }
      }

      return meshTopology;
    }
} // namespace sofa::rigidbodydynamics
