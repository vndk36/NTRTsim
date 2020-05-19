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
 * @file StepwiseController.cpp
 * @brief Implementation of StepwiseController. Based on Ultra-Spines HorizontalSpineController
 * @author Jonathan Bruce
 * $Id$
 */

// This module
#include "SimpleController.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "helpers/FileHelpers.h"
#include <stdexcept>

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
SimpleController::SimpleController(float waterHeight, 
                          std::vector<std::string> tagsToControl) :
  m_tagsToControl(tagsToControl),
  m_waterHeight(waterHeight),
  m_timePassed(0.0),
	m_count(0)
{ 

}

/**
 * The initializeActuators method is call in onSetup to put pointers to
 * specific actuators in the rigidWithTags array, as well as store the initial
 * rest lengths in the initialRL map.
 */
void SimpleController::initializeActuators(TensegrityModel& subject,
						    std::string tag) {
  //DEBUGGING
  std::cout << "Finding rigidBodies with the tag: " << tag << std::endl;
  // Pick out the actuators with the specified tag
  std::vector<tgBaseRigid*> foundRigidBodies = subject.find<tgBaseRigid>(tag);
  std::cout << "The following rigidBodies were found and will have forces " 
            "applied to them be controlled: "
	        << std::endl;
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  rigidWithTags.insert( rigidWithTags.end(), foundRigidBodies.begin(),
			 foundRigidBodies.end() );
}

/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void SimpleController::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up the HorizontalSpine controller." << std::endl;
  //	    << "Finding cables with tags: " << m_tagsToControl
  //	    << std::endl;
  // rigidWithTags = {};
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for( it = m_tagsToControl.begin(); it < m_tagsToControl.end(); it++ ) {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }

  for (std::size_t i = 0; i < rigidWithTags.size(); i ++) {
    rigidWithTags[i]->getPRigidBody()->setActivationState(DISABLE_DEACTIVATION);
  }

  std::cout << "Finished setting up the controller." << std::endl;
}

void SimpleController::onStep(TensegrityModel& subject, double dt)
{
  // Here we take into consideration that only 0.5 radius shere are used
  const float sphere_volume = 0.5236;
  const float water_density = 1025;
  const float b_force       = sphere_volume * water_density * 9.81;
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, if it's passed the time to start the controller,
  // For each cable, check if its rest length is past the minimum,
  // otherwise adjust its length according to m_rate and dt.

  btVector3 force1;
  for (std::size_t i = 0; i < rigidWithTags.size(); i ++) {
      
      // Get current position of the object to compute the force
      // glm::vec3   tempPosition;

      rigidWithTags[i]->getPRigidBody()->setActivationState(DISABLE_DEACTIVATION);
      btTransform trans;
      rigidWithTags[i]->getPRigidBody()->getMotionState()->getWorldTransform (trans);
      currentWaterDepth = -(trans.getOrigin().getY()-m_waterHeight);
      
      std::cout << "Position " << m_tagsToControl[i]
                << " : " << trans.getOrigin().getX()
                << "X, " << trans.getOrigin().getY()
                << "Y, " << trans.getOrigin().getZ()
                << "Z" << std::endl;

      if(currentWaterDepth > 0.0){
        btVector3 force(btScalar(0.), btScalar(b_force), btScalar(0.)); // force is a btVector3
        btVector3 force_pos1(btScalar(5.), btScalar(0.), btScalar(0.)); // force is a btVector3
        //std::cout << "Controller step Force will be applied to " << m_tagsToControl[i] << std::endl;
        rigidWithTags[i]->getPRigidBody()->setDamping(btScalar (.7), btScalar (.3));
        rigidWithTags[i]->getPRigidBody()->applyForce(force,force_pos1);  
      }
      else
      {
        rigidWithTags[i]->getPRigidBody()->setDamping(btScalar (0.0), btScalar (0.0));
      }
  }
  
}