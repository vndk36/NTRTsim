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

// Constructor assigns variables, opens the debug file if DEBUGOUTPUT is on and
buoyancySimulator::buoyancySimulator(float water_height, 
                          std::vector<std::string> tags_to_control) :
  tags_to_control_(tags_to_control),
  water_height_(water_height),
  time_passed_(0.0)
{  
  if(DEBUGOUTPUT)
  { 
    // opens an existing csv file or creates a new file. 
    fout_.open("/Data.csv", std::ios::out | std::ios::app); 
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
 * specific rods in the rod_with_tags_ array.
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
  std::vector<tgRod*> found_rigid_bodies = subject.find<tgRod>(tag);
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  rod_with_tags_.insert( rod_with_tags_.end(), found_rigid_bodies.begin(),
			 found_rigid_bodies.end() );
}

/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void buoyancySimulator::onSetup(TensegrityModel& subject)
{
  std::vector<std::string>::iterator it;
  for( it = tags_to_control_.begin(); it < tags_to_control_.end(); it++ ) {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }

  for (std::size_t i = 0; i < rod_with_tags_.size(); i ++) {
    rod_with_tags_[i]->getPRigidBody()->setActivationState(DISABLE_DEACTIVATION);
  }

  std::cout << "Finished setting up the controller." << "\n";
}

void buoyancySimulator::onStep(TensegrityModel& subject, double dt)
{
  // Water density at sea level
  const float water_density = 1025;

  // Increment the time passed to match  current time.
  time_passed_ += dt;
  
  /** Buoyancy force that will be used at each en of a rod. 
   * Will be recompute at each object. Used static to avoid recreating it each 
   * time. */
  static double tmp_b_force [2];

  /** Pointer to an array of two values. One for each end of the rod */
  static double *tmp_curr_mass;

  /** End point struct used to position information for rods */
  static tgRod::endPoints tmp_end_point_pos;

  /** Loop that goes throught all the rod_with_tags and apply the buoyancy force */
  for (std::size_t i = 0; i < rod_with_tags_.size(); i ++) 
  {

    //rod_with_tags_[i]->getPRigidBody()->setActivationState(DISABLE_DEACTIVATION);

    tmp_end_point_pos = rod_with_tags_[i]->endPointFinder();

    int nb_end_points = tmp_end_point_pos.absolute_pos.size();
    tmp_b_force[0] = ((rod_with_tags_[i]->getVolume())*water_density*9.81)/double(nb_end_points);

    for (std::size_t j = 0; j < nb_end_points; j ++)
    {
      /** 
       * Compute the std buoyancy force frome the volume of the rod itself. 
       * Then is divided by 2 to get one force per end. And finaly the delta force
       * from the BCU is added or substracted 
       */
      current_water_depth_pos1_ = -(tmp_end_point_pos.absolute_pos[j].getY()-water_height_);
      tmp_b_force[0] = ((rod_with_tags_[i]->getVolume())*water_density*9.81)/double(nb_end_points);
      tmp_curr_mass = rod_with_tags_[i]->getMassBCU();

      if(current_water_depth_pos1_ > 0.0){
      tmp_b_force[1] = tmp_b_force[0] + (rod_with_tags_[i]->mass()-tmp_curr_mass[j]);
      btVector3 force(btScalar(0.), btScalar(tmp_b_force[1]), btScalar(0.)); // force is a btVector3
      rod_with_tags_[i]->getPRigidBody()->setDamping(btScalar (.7), btScalar (.5));
      rod_with_tags_[i]->getPRigidBody()->applyForce(force,tmp_end_point_pos.relative_pos[j]); 

      if(DEBUG)
      {
        std::cout << "Controller step Force will be applied to " <<
        tmp_b_force[0] << "   " << rod_with_tags_[i]->mass() << tags_to_control_[i] << std::endl;
      }
      }
      else
      {
        rod_with_tags_[i]->getPRigidBody()->setDamping(btScalar (0), btScalar (0));
      }
    }
    /* 
    if(DEBUG)
    {
      std::cout << tags_to_control_[i] <<
                "  Pos1 x " << tmp_end_point_pos.absolute_pos[0].getX() <<
                 " y " <<       tmp_end_point_pos.absolute_pos[0].getY() <<
                 " z " <<       tmp_end_point_pos.absolute_pos[0].getZ() <<
                 " Pos2 x " <<  tmp_end_point_pos.absolute_pos[1].getX() <<
                 " y " <<       tmp_end_point_pos.absolute_pos[1].getY() <<
                 " z " <<       tmp_end_point_pos.absolute_pos[1].getZ() << "\n";


      if(DEBUGOUTPUT)
      {
        fout_ << time_passed_ <<
              "," << tmp_end_point_pos.absolute_pos[0].getX() <<
              "," << tmp_end_point_pos.absolute_pos[0].getY() <<
              "," << tmp_end_point_pos.absolute_pos[0].getZ() <<
              "," << tmp_end_point_pos.absolute_pos[1].getX() <<
              "," << tmp_end_point_pos.absolute_pos[1].getY() <<
              "," << tmp_end_point_pos.absolute_pos[1].getZ() << "\n";
      }
    } */
  } 
}
