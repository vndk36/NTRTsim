/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file tgRod.cpp
 * @brief Contains the definitions of members of class tgRod
 * @author Ryan Adams
 * $Id$
 */

// This module
#include "tgRod.h"
#include "tgModelVisitor.h"
// The Bullet Physics library
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <sstream> //for the tgSenseable methods.
#include <iostream> //for strings.
#include <fstream>

#define DEBUG 0

tgRod::Config::Config(double r, double d,
                        double f, double rf, double res) :
  radius(r),
  density(d),
  friction(f),
  rollFriction(rf),
  restitution(res)
{
        if (density < 0.0) { throw std::range_error("Negative density"); }
        if (radius < 0.0)  { throw std::range_error("Negative radius");  }
        if (friction < 0.0)  { throw std::range_error("Negative friction");  }
        if (rollFriction < 0.0)  { throw std::range_error("Negative roll friction");  }
        if (restitution < 0.0)  { throw std::range_error("Negative restitution");  }
        if (restitution > 1.0)  { throw std::range_error("Restitution > 1");  }
    // Postcondition
    assert(density >= 0.0);
    assert(radius >= 0.0);
    assert(friction >= 0.0);
    assert((rollFriction >= 0.0) && (rollFriction <= 1.0));
    assert((restitution >= 0.0) && (restitution <= 1.0));
}

tgRod::tgRod(btRigidBody* pRigidBody, 
                const tgTags& tags,
                const double length) : 
  tgBaseRigid(pRigidBody, tags),
  m_length(length)
{
  if (pRigidBody == NULL)
  {
          throw std::invalid_argument("Pointer to btRigidBody is NULL");
  }

  // Postcondition
  assert(invariant());
  assert(m_pRigidBody == pRigidBody);
}

tgRod::~tgRod() { }

void tgRod::onVisit(const tgModelVisitor& v) const
{
    v.render(*this);
    
}

void tgRod::teardown()
{
  // Set body to NULL, calls teardown on children
  tgBaseRigid::teardown();
  
  // Postcondition
  // This does not preserve the invariant
}

bool tgRod::invariant() const
{
  return
    (m_pRigidBody != NULL) &&
    (m_mass >= 0.0) &&
    (m_length >= 0.0);
}

/**
 * Finds and returns the end points of the rod. It returns an endpoint struct 
 * that contains relative positions from the rod center point and absolut
 * positions from the world referneces. 
 *
 * @param[in] void.
 * @return endpoint structure that has the relative and absolute positon of the
 * end points.
 */
tgBaseRigid::endPoints tgRod::getEndPoints (void)
{

  m_com = btVector3(centerOfMass().x(), centerOfMass().y(), centerOfMass().z());
  m_rot = btMatrix3x3(m_pRigidBody->getOrientation());
  
  if(DEBUG)
  {
    btVector3 rot2 = m_rot.getRow(0);
    std::cout << rot2.getX() << " " << rot2.getY() << " " << rot2.getZ() <<"\n";
    rot2 = m_rot.getRow(1);
    std::cout << rot2.getX() << " " << rot2.getY() << " " << rot2.getZ() <<"\n";
    rot2 = m_rot.getRow(2);
    std::cout << rot2.getX() << " " << rot2.getY() << " " << rot2.getZ() <<"\n";
  }
  /**
   * Use of the orientation matrix of the object and the initial orientation 
   * vector to have the relative and absolute position of the end points
   */
  /* m_endPointPos.relative_pos[0] = (m_rot * m_orig1);
  m_endPointPos.relative_pos[1] = (m_rot * m_orig2);
  m_endPointPos.absolute_pos[0] = m_endPointPos.relative_pos[0] + m_com; 
  m_endPointPos.absolute_pos[0] = m_endPointPos.relative_pos[1] + m_com; */

  m_endPointPos.absolute_pos.clear();
  m_endPointPos.relative_pos.clear();
  m_endPointPos.relative_pos.push_back(m_rot * m_orig1);
  m_endPointPos.relative_pos.push_back(m_rot * m_orig2);
  m_endPointPos.absolute_pos.push_back(m_endPointPos.relative_pos[0] + m_com); 
  m_endPointPos.absolute_pos.push_back(m_endPointPos.relative_pos[1] + m_com);

  return m_endPointPos;
}


/** 
 * Use the to starting node position (From and To) to compute the main axis of
 * the rod. 
 * @param[in] from and to which are btVector3 from the pair of node used to 
 * construct the rod.
 * @return void.
 */
void tgRod::mainAxisSetup(btVector3 from, btVector3 to)
{
  
  m_orig1 = from-(from-((from - to)/2.0));
  m_orig2 = to-(from-((from-to)/2.0));


/**
   * As we get the inverted inertia diag, the "main" axis is the axis with the 
   * biggest value. As the rod is a cylindre, both of the other values should 
   * be identitcal
   */
  /* btVector3 inv_inertial = getPRigidBody()->getInvInertiaDiagLocal();

  if( inv_inertial.getX() > inv_inertial.getY() && 
      inv_inertial.getX() > inv_inertial.getZ())
  {
    std::cout << "X"<< "\n";
    m_orig1 = btVector3((m_length/2.0), 0.0, 0.0);
    m_orig2 = btVector3(-(m_length/2.0), 0.0, 0.0);
  }

  if( inv_inertial.getY() > inv_inertial.getX() && 
      inv_inertial.getY() > inv_inertial.getZ())
  {
    std::cout << "Y"<< "\n";
    m_orig1 = btVector3(0.0, -(m_length/2.0), 0.0);
    m_orig2 = btVector3(0.0, (m_length/2.0), 0.0);
  }

  if( inv_inertial.getZ() > inv_inertial.getX() && 
      inv_inertial.getZ() > inv_inertial.getY())
  {
    std::cout << "Z"<< "\n";
    m_orig1 = btVector3(0.0, 0.0, (m_length/2.0));
    m_orig2 = btVector3(0.0, 0.0, -(m_length/2.0));
  }  */
}
