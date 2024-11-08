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

#include <string>

namespace sofa::rigidbodydynamics
{
  enum class NoCollisionReason 
  {
    ADJACENT = 0,
    NEVER = 1,
    DEFAULT = 2
  };

  struct CollisionPair
  {
    unsigned int id;
    std::string link1;
    std::string link2;
    NoCollisionReason reason;
  };

  struct CollisionPairComparator
  {
      bool operator() (const CollisionPair &a, const CollisionPair &b) const
      {
        return std::min(a.link1, a.link2) < std::min(b.link1, b.link2);
      }
  };

} // namespace sofa::rigidbodydynamics
