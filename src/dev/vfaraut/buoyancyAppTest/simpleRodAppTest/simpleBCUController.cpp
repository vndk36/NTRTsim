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
 * @file simpleBCUController.cpp
 * @brief Implementation of LinkedController. Based on Ultra-Spines HorizontalSpineController
 * @author Victor Faraut
 * $Id$
 */

// This module
#include "simpleBCUController.h"
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

// Forward declaration
void setMassRodSides(tgRod* p_rodTags, std::vector<double> mass);


// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
simpleBCUController::simpleBCUController(float waterHeight, 
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
void simpleBCUController::initializeActuators(TensegrityModel& subject,
						    std::string tag) {
  //DEBUGGING
  std::cout << "Finding rigidBodies with the tag: " << tag << "\n";
  // Pick out the actuators with the specified tag
  std::vector<tgRod*> p_foundRigidBodies = subject.find<tgRod>(tag);
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
void simpleBCUController::onSetup(TensegrityModel& subject)
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

void simpleBCUController::onStep(TensegrityModel& subject, double dt)
{
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, if it's passed the time to start the controller,
  // For each cable, check if its rest length is past the minimum,
  // otherwise adjust its length according to m_rate and dt.

  if( m_timePassed > 0.0 ) {
    
    for (std::size_t i = 0; i < p_rigidWithTags.size(); i ++) {
        m_mass.clear();

        if(m_timeBCU > 0.0 && m_timeBCU <= 10.0){
          m_mass.push_back(p_rigidWithTags.at(i)->getMassBCU()[0] + (dt*0.10));
          m_mass.push_back(p_rigidWithTags.at(i)->getMassBCU()[1] - (dt*0.10));
          setMassRodSides(p_rigidWithTags.at(i), m_mass);
        }
        if(m_timeBCU > 20.0 && m_timeBCU <= 30.0){
          m_mass.push_back(p_rigidWithTags.at(i)->getMassBCU()[0] - (dt*0.10));
          m_mass.push_back(p_rigidWithTags.at(i)->getMassBCU()[1] + (dt*0.10));
          setMassRodSides(p_rigidWithTags.at(i), m_mass);

        }
        if(m_timeBCU > 40.0){
          m_timeBCU = 0.0;
        }

        

      m_timeBCU += dt;

    
    }
  }
}
void setMassRodSides(tgRod* p_rodTags, std::vector<double> mass)
{
  std::cout << mass.size() << " \n";
  for (std::size_t i = 0; i < mass.size(); i ++)
  {
    if (mass.at(i) >= p_rodTags->getMassBCUMin())
    {
      if (mass.at(i) <= p_rodTags->getMassBCUMax())
      {
        //std::cout << "Value set " << mass[i] << "\n";
        p_rodTags->setMassBCU(mass.at(i),i);
      }
      else
      {
        //std::cout << "Max value reached " << mass[i] << "\n";
        p_rodTags->setMassBCU(p_rodTags->getMassBCUMax(),i);

      }
      
    }
    else
    {
      //std::cout << "Min value reached " << mass[i] << "\n";
      p_rodTags->setMassBCU(p_rodTags->getMassBCUMin(),i);
    }
  }
}
