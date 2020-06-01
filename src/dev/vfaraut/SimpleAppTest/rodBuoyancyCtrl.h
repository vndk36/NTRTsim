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

#ifndef ROD_BUOYANCY_CTRL_H
#define ROD_BUOYANCY_CTRL_H

/**
 * @file rodBuoyancyCtrl.h
 * @brief Contains the definition of class rodBuoyancyCtrl. Based on Ultra-Spines HorizontalSpineController
 * @author Victor Faraut
 * $Id$
 */

// The NTRT core library
#include "core/tgObserver.h"
#include "core/tgTags.h"
#include "core/tgBaseRigid.h" 

//Bullet library
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"

// The C++ standard library
#include <string>
#include <vector>
#include <map>

// Forward declarations
class TensegrityModel;
class tgBasicActuator;

/**
  * Array with the relation between tags and mass of the object. This is used 
  * to have the buoyancy force that is applied on the object
  */
struct bodyMass{
  tgBaseRigid* tag;
  double weight;    
};

struct rodEnds{
  btVector3 pos1;
  btVector3 pos2;    
};

/**
 * A controller to apply the length change in the cables of SUPERball V2 model
 */
class rodBuoyancyCtrl : public tgObserver<TensegrityModel>
{
public:

  /**
   * Construct a StepwiseController.
   * @param[in] startTime, a double that determines when the controller
   * begins its motion, how many seconds after the simulation starts.
   * @param[in] minLength, a double that is the percent of the initial length
   * that this controller will reduce down to. E.g., if minLength = 0.25,
   * controller will act until the rest length of the cables is 25% of initial.
   * @param[in] rate, the rate at which the rest length of the cables will be
   * changed. Expressed in meters/sec.
   * @param[in] tagsToControl, a vector (array) of strings, which is a list of the
   * tags of all the
   * cables upon which to act. All the cables which have a tag in this list of tags
   * will be acted upon by this controller.
   */
  rodBuoyancyCtrl(float waterHeight, std::vector<std::string> tagsToControl);

  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~rodBuoyancyCtrl() { }

  /**
   * Apply the controller. On setup, adjust the cable
   * lengths one time.
   * @param[in] subject - the TensegrityModel that is being controlled. Must
   * have a list of allMuscles populated
   */
  virtual void onSetup(TensegrityModel& subject);

  /**
   * The onStep method is not used for this controller.
   * @param[in] subject - the TensegrityModel that is being controlled. Must
   * have a list of allMuscles populated
   * @param[in] dt, current timestep must be positive
   */
  virtual void onStep(TensegrityModel& subject, double dt);

protected:

void initializeActuators(TensegrityModel& subject, std::string tag);
rodEnds findRodEndPoints(tgBaseRigid* rod);


private:
  /**
   * The private variables for each of the values passed in to the constructor.
   */
  std::vector<std::string> m_tagsToControl;

  /**
   * Need an accumulator variable to determine when to start the controller.
   */
  double m_timePassed;
  double m_previousTime;
  int m_count;
  float m_waterHeight;
  float currentWaterDepth;
  btScalar buoyancyForce;
  
  /**
   * Array with the relation between tags and mass of the object. This is used 
   * to have the buoyancy force that is applied on the object
   */
  struct bodyMass{
    tgBaseRigid* tag;
    double weight;    
  };

  
  /**
   * A list of all the actuators to control. This is populated in onSetup
   * by using m_tagsToControl.
   */
  std::vector<tgBaseRigid*> rigidWithTags;
  std::vector<rodEnds> rodWithTagsEnds;

  

};

#endif // ROD_BUOYANCY_CTRL_H