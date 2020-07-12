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
 * @brief A "controller" that simulates buoyancy for rod objects. All the 
 * details are available at :
 * https://docs.google.com/document/d/1O8zBkGAI20gAAwfgTX-zJSVDS1vTR_Rzs0rctpgP9LQ/edit?usp=sharing
 * @author Victor Faraut
 * $Id$
 */

// This module
#include "buoyancySimulator.h"

// This application
#include "yamlbuilder/TensegrityModel.h"

// This library
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

// Constructor assigns variables, opens the debug file if DEBUGOUTPUT is on.
buoyancySimulator::buoyancySimulator(float waterHeight, 
                          std::vector<std::string> tagsToControl) :
  m_tagsToControl(tagsToControl),
  m_waterHeight(waterHeight),
  m_timePassed(0.0)
{  
  if(DEBUGOUTPUT)
  { 
    // opens an existing csv file or creates a new file. 
    m_fout.open("/Data.csv", std::ios::out | std::ios::app); 
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
 * specific rods in the m_rodWithTags array.
 */
void buoyancySimulator::initializeActuators(TensegrityModel& subject,
						    std::string tag) {
  if(DEBUG)
  {
    std::cout << "Finding rigidBodies with the tag: " << tag << "\n";
  }  

  /** 
   * Pick out the actuators with the specified tag. 
   * They could be multiple with the same tag.
   */
  std::vector<tgRod*> foundRigidBodies = subject.find<tgRod>(tag);
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  m_rodWithTags.insert( m_rodWithTags.end(), foundRigidBodies.begin(),
			 foundRigidBodies.end() );
}

/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void buoyancySimulator::onSetup(TensegrityModel& subject)
{
  std::vector<std::string>::iterator it;
  for( it = m_tagsToControl.begin(); it < m_tagsToControl.end(); it++ ) {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }

  for (std::size_t i = 0; i < m_rodWithTags.size(); i ++) {
    m_rodWithTags[i]->getPRigidBody()->setActivationState(DISABLE_DEACTIVATION);
  }

  std::cout << "Finished setting up the controller." << "\n";
}

void buoyancySimulator::onStep(TensegrityModel& subject, double dt)
{
  // Water density at sea level
  const float waterDensity = 1.025;

  // Increment the time passed to match  current time.
  m_timePassed += dt;
  
  /** Buoyancy force that will be used at each en of a rod. 
   * Will be recompute at each object. Used static to avoid recreating it each 
   * time. */
  static double tmpBForce [2];



  /** End point struct used to position information for rods */
  static tgBaseRigid::endPoints tmpEndPointPos;

  /** Loop that goes throught all the rod_with_tags and apply the buoyancy force */
  for (std::size_t i = 0; i < m_rodWithTags.size(); i ++) 
  {

    //std::cout << "mass " << m_rodWithTags[i]->mass() << "\n";

    //m_rodWithTags[i]->getPRigidBody()->setActivationState(DISABLE_DEACTIVATION);

    tmpEndPointPos = m_rodWithTags[i]->getEndPoints();

    /** Pointer to an array of two values. One for each end of the rod */
    std::vector<double> tmpCurrMass = m_rodWithTags[i]->getMassBCU();

    int nb_end_points = tmpEndPointPos.absolute_pos.size();
    tmpBForce[0] = ((m_rodWithTags[i]->getVolume())*waterDensity*9.81)/double(nb_end_points);

    std::cout << "One Step \n";

    for (std::size_t j = 0; j < nb_end_points; j ++)
    {
      /** 
       * Compute the std buoyancy force frome the volume of the rod itself. 
       * Then is divided by 2 to get one force per end. And finaly the delta force
       * from the BCU is added or substracted 
       */
      m_currentWaterDepthPos1 = -(tmpEndPointPos.absolute_pos[j].getY()-m_waterHeight);
      tmpBForce[0] = ((m_rodWithTags[i]->getVolume())*waterDensity*98.1)/double(nb_end_points);
      //tmpCurrMass = m_rodWithTags[i]->getMassBCU();
      std::cout << "mass is " << m_rodWithTags[i]->mass() << " BCU mass is " << tmpCurrMass[0] << "\n";
      std::cout << "mass is " << m_rodWithTags[i]->mass() << " BCU mass is " << tmpCurrMass[1] << "\n";

      if(m_currentWaterDepthPos1 > 0.0){
      tmpBForce[1] = tmpBForce[0] + (m_rodWithTags[i]->mass()-tmpCurrMass[j]);
      btVector3 force(btScalar(0.), btScalar(tmpBForce[1]), btScalar(0.)); // force is a btVector3
      m_rodWithTags[i]->getPRigidBody()->setDamping(btScalar (.7), btScalar (.5));
      m_rodWithTags[i]->getPRigidBody()->applyForce(force,tmpEndPointPos.relative_pos[j]); 

      if(DEBUG)
      {
        std::cout << "Controller step Force will be applied to " << 
                      m_tagsToControl[i] << std::endl;
        std::cout << m_tagsToControl[i] <<
                "  Pos x " << tmpEndPointPos.absolute_pos[j].getX() <<
                 " y " <<     tmpEndPointPos.absolute_pos[j].getY() <<
                 " z " <<     tmpEndPointPos.absolute_pos[j].getZ() << "\n";
      }
      }
      else
      {
        m_rodWithTags[i]->getPRigidBody()->setDamping(btScalar (0), btScalar (0));
      }
    }
    /* 
    if(DEBUG)
    {
      std::cout << m_tagsToControl[i] <<
                "  Pos1 x " << tmpEndPointPos.absolute_pos[0].getX() <<
                 " y " <<       tmpEndPointPos.absolute_pos[0].getY() <<
                 " z " <<       tmpEndPointPos.absolute_pos[0].getZ() <<
                 " Pos2 x " <<  tmpEndPointPos.absolute_pos[1].getX() <<
                 " y " <<       tmpEndPointPos.absolute_pos[1].getY() <<
                 " z " <<       tmpEndPointPos.absolute_pos[1].getZ() << "\n";


      if(DEBUGOUTPUT)
      {
        m_fout << m_timePassed <<
              "," << tmpEndPointPos.absolute_pos[0].getX() <<
              "," << tmpEndPointPos.absolute_pos[0].getY() <<
              "," << tmpEndPointPos.absolute_pos[0].getZ() <<
              "," << tmpEndPointPos.absolute_pos[1].getX() <<
              "," << tmpEndPointPos.absolute_pos[1].getY() <<
              "," << tmpEndPointPos.absolute_pos[1].getZ() << "\n";
      }
    } */
  } 
}
