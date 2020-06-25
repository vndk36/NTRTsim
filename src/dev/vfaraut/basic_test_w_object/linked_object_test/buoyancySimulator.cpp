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
 * @file buoyancySimulator.cpp
 * @brief Implementation of a Buoyancy simulation for rod objects in NTRT 
 * For more information on the implementation see on this drive link: 
 *  
 * @author Victor Faraut
 * $Id$
 */

// This module
#include "buoyancySimulator.h"
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

#define DEBUG 0
#define DEBUGOUTPUT 0

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
buoyancySimulator::buoyancySimulator(float waterHeight, 
                          std::vector<std::string> tagsToControl) :
  m_tagsToControl(tagsToControl),
  m_waterHeight(waterHeight),
  m_timePassed(0.0),
	m_count(0)
{ 
  // file pointer 
  if(DEBUG)
  { 
    // opens an existing csv file or creates a new file. 
    m_fout.open("/mnt/c/Users/victo/Desktop/Data.csv", std::ios::out | std::ios::app); 

    // Insert the data to file 
    // fout << 3 << "\n";
  }

  /* try {
        boost::asio::io_service io_service;
        UDPServer server{io_service};
        io_service.run();
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
    } */
}

/**
 * The initializeActuators method is call in onSetup to put pointers to
 * specific actuators in the rigidWithTags array, as well as store the initial
 * rest lengths in the initialRL map.
 */
void buoyancySimulator::initializeActuators(TensegrityModel& subject,
						    std::string tag) {
  //DEBUGGING
  std::cout << "Finding rigidBodies with the tag: " << tag << "\n";
  // Pick out the actuators with the specified tag
  std::vector<tgRod*> foundRigidBodies = subject.find<tgRod>(tag);
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
void buoyancySimulator::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up the HorizontalSpine controller." << "\n";
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

  std::cout << "Finished setting up the controller." << "\n";
}

void buoyancySimulator::onStep(TensegrityModel& subject, double dt)
{
  // Here we take into consideration that only 0.5 radius shere are used
  const float water_density = 1025;
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, if it's passed the time to start the controller,
  // For each cable, check if its rest length is past the minimum,
  // otherwise adjust its length according to m_rate and dt.

  static double b_force [2];
  static double *curr_mass;
  static tgRod::endPoints endPointPos;

  static btVector3 test_I;
  for (std::size_t i = 0; i < rigidWithTags.size(); i ++) {
      
    // Get current position of the object to compute the force
    // glm::vec3   tempPosition;

    rigidWithTags[i]->getPRigidBody()->setActivationState(DISABLE_DEACTIVATION);
    btTransform trans;
    // rigidWithTags[i]->getPRigidBody()->getMotionState()->getWorldTransform (trans);
    // currentWaterDepth = -(trans.getOrigin().getY()-m_waterHeight);

    endPointPos = rigidWithTags[i]->endPointFinder();
    currentWaterDepthPos1 = -(endPointPos.absolutePos1.getY()-m_waterHeight);
    currentWaterDepthPos2 = -(endPointPos.absolutePos2.getY()-m_waterHeight);

    test_I = rigidWithTags[i]->getPRigidBody()->getInvInertiaDiagLocal();
    
    if(DEBUG)
    {
      std::cout << m_tagsToControl[i] <<
                "  Pos1 x " << endPointPos.absolutePos1.getX() <<
                 " y " <<       endPointPos.absolutePos1.getY() <<
                 " z " <<       endPointPos.absolutePos1.getZ() <<
                 " Pos2 x " <<  endPointPos.absolutePos2.getX() <<
                 " y " <<       endPointPos.absolutePos2.getY() <<
                 " z " <<       endPointPos.absolutePos2.getZ() << "\n";


      if(DEBUGOUTPUT)
      {
        m_fout << m_timePassed <<
              "," << endPointPos.absolutePos1.getX() <<
              "," << endPointPos.absolutePos1.getY() <<
              "," << endPointPos.absolutePos1.getZ() <<
              "," << endPointPos.absolutePos2.getX() <<
              "," << endPointPos.absolutePos2.getY() <<
              "," << endPointPos.absolutePos2.getZ() << "\n";
      }
    }


    b_force[0] = ((rigidWithTags[i]->getVolume())*water_density*9.81)/2.0;
    b_force[1] = b_force[0];
    curr_mass = rigidWithTags[i]->getMassBCU();

    if(currentWaterDepthPos1 > 0.0){
      b_force[0] = b_force[0] + (rigidWithTags[i]->mass()-curr_mass[0]);
      btVector3 force(btScalar(0.), btScalar(b_force[0]), btScalar(0.)); // force is a btVector3
      rigidWithTags[i]->getPRigidBody()->setDamping(btScalar (.7), btScalar (.5));
      rigidWithTags[i]->getPRigidBody()->applyForce(force,endPointPos.relativePos1); 

      if(DEBUG)
      {
        std::cout << "Controller step Force will be applied to " <<
        b_force[0] << "   " << rigidWithTags[i]->mass() << m_tagsToControl[i] << std::endl;
      }
    }
    else
    {
      rigidWithTags[i]->getPRigidBody()->setDamping(btScalar (0), btScalar (0));
    }


    if(currentWaterDepthPos2 > 0.0){
      b_force[1] = b_force[1] + (rigidWithTags[i]->mass()-curr_mass[1]);
      btVector3 force(btScalar(0.), btScalar(b_force[1]), btScalar(0.)); // force is a btVector3
      rigidWithTags[i]->getPRigidBody()->setDamping(btScalar (.7), btScalar (.5));
      rigidWithTags[i]->getPRigidBody()->applyForce(force,endPointPos.relativePos2);  
      if(DEBUG)
      {
        std::cout << "Controller step Force will be applied to " <<
        b_force[1] << "   " << rigidWithTags[i]->mass() << m_tagsToControl[i] << std::endl;
      }
    }
    else
    {
      rigidWithTags[i]->getPRigidBody()->setDamping(btScalar (0), btScalar (0));
    }
  } 
}
