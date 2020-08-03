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
 * @file rotBCUController.cpp
 * @brief Implementation of LinkedController. Based on Ultra-Spines HorizontalSpineController
 * @author Victor Faraut
 * $Id$
 */

// This module
#include "rotBCUController.h"
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

#define DEBUG         0
#define DEBUGOUTPUT   0

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
rotBCUController::rotBCUController(std::vector<std::string> tagsToControl) :
  m_tagsToControl(tagsToControl),
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
}

/**
 * The initializeActuators method is call in onSetup to put pointers to
 * specific actuators in the rigidWithTags array, as well as store the initial
 * rest lengths in the initialRL map.
 */
void rotBCUController::initializeActuators(TensegrityModel& subject,
						    std::string tag) {
  //DEBUGGING
  std::cout << "Finding rigidBodies with the tag: " << tag << "\n";
  // Pick out the actuators with the specified tag
  std::vector<tgBaseRigid*> p_foundRigidBodies = subject.find<tgBaseRigid>(tag);
  std::cout << "The following rigidBodies were found and will have forces " 
            "applied to them be controlled: "
	        << std::endl;
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  p_rigidWithTags.insert( p_rigidWithTags.end(), p_foundRigidBodies.begin(),
			 p_foundRigidBodies.end() );
}
/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void rotBCUController::onSetup(TensegrityModel& subject)
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

  std::cout << "Finished setting up the controller." << "\n";
}

void rotBCUController::onStep(TensegrityModel& subject, double dt)
{
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, if it's passed the time to start the controller,
  // For each cable, check if its rest length is past the minimum,
  // otherwise adjust its length according to m_rate and dt.

  //std::cout << "Dt for this step " << dt << "\n" <<
  //          "Case " << m_count << "\n";

  const double rate = 0.1;


  if( m_timePassed > START_TIME) {
    for(int i = 0; i<NB_CONTROLLABLE; i++)
    {
      setMassForOneRigid (i, dt);
    }
      
    if(m_timeBCU > DELTA_T_CASE)
    {
      m_count ++;
      if(m_count >= NB_CASE)
        m_count = 0;
      m_timeBCU = 0.0;
    }

    m_timeBCU += dt;
  }
}

void rotBCUController::setMassForOneRigid (int rigidIdx, double dt)
{
  int tmpCase = 0;

  tmpCase = m_ctrlMatrix[m_count][rigidIdx];

  m_mass.clear();

  switch (tmpCase)
  {
  case 0:
    /* Do nothing */
    break;

  case 1:
    /* Increase both sides */
    m_mass.push_back(p_rigidWithTags.at(rigidIdx)->getMassBCU()[0] + (dt*RATE));
    m_mass.push_back(p_rigidWithTags.at(rigidIdx)->getMassBCU()[1] + (dt*RATE));
    p_rigidWithTags.at(rigidIdx)->setMassBCU(m_mass[0], 0);
    p_rigidWithTags.at(rigidIdx)->setMassBCU(m_mass[1], 1);
    break;

  case 2:
    /* Decrease both sides */
    m_mass.push_back(p_rigidWithTags.at(rigidIdx)->getMassBCU()[0] - (dt*RATE));
    m_mass.push_back(p_rigidWithTags.at(rigidIdx)->getMassBCU()[1] - (dt*RATE));
    p_rigidWithTags.at(rigidIdx)->setMassBCU(m_mass[0], 0);
    p_rigidWithTags.at(rigidIdx)->setMassBCU(m_mass[1], 1);
    break;

  case 3:
    /* increase side 0 and decrease side 1 */
    m_mass.push_back(p_rigidWithTags.at(rigidIdx)->getMassBCU()[0] + (dt*RATE));
    m_mass.push_back(p_rigidWithTags.at(rigidIdx)->getMassBCU()[1] - (dt*RATE));
    p_rigidWithTags.at(rigidIdx)->setMassBCU(m_mass[0], 0);
    p_rigidWithTags.at(rigidIdx)->setMassBCU(m_mass[1], 1);
    break;

  case 4:
    /* increase side 1 and decrease side 0  */
    m_mass.push_back(p_rigidWithTags.at(rigidIdx)->getMassBCU()[0] - (dt*RATE));
    m_mass.push_back(p_rigidWithTags.at(rigidIdx)->getMassBCU()[1] + (dt*RATE));
    p_rigidWithTags.at(rigidIdx)->setMassBCU(m_mass[0], 0);
    p_rigidWithTags.at(rigidIdx)->setMassBCU(m_mass[1], 1);
    break;
  default:
    break;
  }


  //std::cout<< "Mass set for "<<rigidIdx<< " with the case " << tmpCase << "\n"<<
  //          "Mass A = "<< m_mass[0] << "   Mass B = " << m_mass[1] << "\n";

}