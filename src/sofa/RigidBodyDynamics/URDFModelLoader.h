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

#include <sofa/RigidBodyDynamics/config.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataFileName.h>

namespace sofa::rigidbodydynamics
{

  using namespace core::objectmodel;

  class SOFA_RIGIDBODYDYNAMICS_API URDFModelLoader : public BaseObject
  {
  public:
    SOFA_CLASS(URDFModelLoader, BaseObject);

    void setURDFFilename(const std::string &f);
    const std::string &getURDFFilename() /*const*/;

    void setModelDirectory(const std::string &d);
    const std::string &getModelDirectory() /*const*/;

    void setUseFreeFlyerRootJoint(bool useFreeFlyerRootJoint);
    bool getUseFreeFlyerRootJoint();

    void init() override;

    void reinit() override;

    DataFileName d_urdfFilename;
    Data<std::string> d_modelDirectory;
    Data<bool> d_useFreeFlyerRootJoint;
    Data<sofa::type::vector<sofa::type::Vec1d>> d_q0;

  private:
    URDFModelLoader();
  };

} /// namespace sofa::rigidbodydynamics