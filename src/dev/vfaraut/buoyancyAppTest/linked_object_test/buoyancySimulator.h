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

#ifndef BUOYANCY_SIMULATOR_H
#define BUOYANCY_SIMULATOR_H

/**
 * @file buoyancySimulator.h
 * @brief A "controller" that simulates buoyancy for rod objects. All the 
 * details are available at :
 * https://docs.google.com/document/d/1O8zBkGAI20gAAwfgTX-zJSVDS1vTR_Rzs0rctpgP9LQ/edit?usp=sharing
 * @author Victor Faraut
 * $Id$
 */

// The NTRT core library
#include "core/tgObserver.h"
#include "core/tgTags.h"
#include "core/tgBaseRigid.h" 
#include "core/tgRod.h"

// Bullet library
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"

// The C++ standard library
#include <string>
#include <vector>
#include <map>
#include <fstream>

// Forward declarations
class TensegrityModel;
class tgBasicActuator;

/**
 * A controller to apply the length change in the cables of SUPERball V2 model
 */
class buoyancySimulator : public tgObserver<TensegrityModel>
{
public:

  /**
   * Construct the buoyancy simulator controller for the given objects.
   * @param[in] waterHeight, Height of the simulated water. Above this height 
   * the buoyancy force will not be applied.
   * @param[in] tagsToControl, a vector (array) of strings, which is a list of 
   * the tags of all the rods upon which to act.
   */
  buoyancySimulator(float waterHeight, std::vector<std::string> tagsToControl);

  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~buoyancySimulator() { }

  /**
   * Apply the controller. On setup, adjust the cable
   * lengths one time.
   * @param[in] subject - the TensegrityModel that is being controlled. Must
   * have a list of all tg objects controlled
   */
  virtual void onSetup(TensegrityModel& subject);

  /**
   * The onStep method is used to add the buoyancy force each time the 
   * simulation gets recomputed.
   * @param[in] subject - the TensegrityModel that is being controlled. Must
   * have a list of all tg objects controlled
   * @param[in] dt, current timestep must be positive
   */
  virtual void onStep(TensegrityModel& subject, double dt);

protected:

void initializeActuators(TensegrityModel& subject, std::string tag);


private:
  /**
   * Vector that stores the tags of all the tgRod that have buoyancy applied 
   */
  std::vector<std::string> m_tagsToControl;

  /**
   * A list of all the actuators to control. This is populated in onSetup
   * by using m_tagsToControl.
   */
  std::vector<tgRod*> m_rodWithTags;


  // time passed used for timestep on logs
  double m_timePassed;

  // stores the water height to be compared to point position
  float m_waterHeight;

  // Current position of the end point of the rod. This is changed for each rod
  double m_currentWaterDepthPos1;
  double m_currentWaterDepthPos2;

  // file pointer for log
  std::ofstream m_fout; 

};

#endif // BUOYANCY_SIMULATOR_H