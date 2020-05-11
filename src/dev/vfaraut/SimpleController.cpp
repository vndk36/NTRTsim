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
  std::vector<tgBaseRigid*> foundRgidBodies = subject.find<tgBaseRigid>(tag);
  std::cout << "The following rigidBodies were found and will have forces " 
            "applied to them be controlled: "
	        << std::endl;
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  rigidWithTags.insert( rigidWithTags.end(), foundRgidBodies.begin(),
			 foundRgidBodies.end() );
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
  rigidWithTags = {};
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for( it = m_tagsToControl.begin(); it < m_tagsToControl.end(); it++ ) {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }
  std::cout << "Finished setting up the controller." << std::endl;
}

void SimpleController::onStep(TensegrityModel& subject, double dt)
{
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, if it's passed the time to start the controller,
  // For each cable, check if its rest length is past the minimum,
  // otherwise adjust its length according to m_rate and dt.
  for (std::size_t i = 0; i < rigidWithTags.size(); i ++) {
      
      // Get current position of the object to compute the force
      // glm::vec3   tempPosition;

      btTransform trans;
      rigidWithTags[i]->getPRigidBody()->getMotionState()->getWorldTransform (trans);

      if(trans.getOrigin().getY()< m_waterHeight){
        btVector3 force(btScalar(0.), btScalar(350.), btScalar(0.)); // force is a btVector3
        std::cout << "Controller step Force will be applied to " << m_tagsToControl[i] << std::endl;
        rigidWithTags[i]->getPRigidBody()->setDamping(btScalar (10.0), btScalar (10.0));
        rigidWithTags[i]->getPRigidBody()->applyCentralForce(force);  
      }
      else
      {
        rigidWithTags[i]->getPRigidBody()->setDamping(btScalar (0.0), btScalar (0.0));
      }
  }
  
}