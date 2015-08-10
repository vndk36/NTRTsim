/*
* Copyright © 2015, United States Government, as represented by the
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
* @file AppCrossArm.cpp
* @brief Contains the definition function main() for the tensegrity arm based upon the designs of Graham Scarr, but with enhanced control on yaw movement
* application.
* $Id$
*/
// This application
#include "CrossArmModel.h"
#include "CrossArmController.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
/**
* The entry point.
* @param[in] argc the number of command-line arguments
* @param[in] argv argv[0] is the executable name
* @return 0
*/
int main(int argc, char** argv) {
	std::cout << "AppCrossArm" << std::endl;
	// First create the ground and world
	// Determine the angle of the ground in radians. All 0 is flat
	const double yaw = 0.0;
	const double pitch = 0.0;
	const double roll = 0.0;
	const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));

	// the world will delete this
	tgBoxGround* ground = new tgBoxGround(groundConfig);
	const tgWorld::Config config(98.1); // gravity, cm/sec^2 Use this to adjust length scale of world.
	// Note, by changing the setting below from 981 to 98.1, we've
	// scaled the world length scale to decimeters not cm.
	tgWorld world(config, ground);

	// Second create the view
	const double timestep_physics = 1.0 / 60.0 / 10.0; // Seconds
	const double timestep_graphics = 1.f /60.f; // Seconds
	#if(1)
	tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
	#else
	tgSimView view(world, timestep_physics, timestep_graphics);
	#endif

	// Third create the simulation
	tgSimulation simulation(view);

	// Fourth create the models with their controllers and add the models to the simulation
	CrossArmModel* const myModel = new CrossArmModel();

	// Fifth, select the controller to use. Uncomment desired controller.
	CrossArmController* const pTC = new CrossArmController(2.5, timestep_physics);

	myModel->attach(pTC);
	simulation.addModel(myModel);

	// Run until the user stops
	simulation.run();
	simulation.reset();

	//Teardown is handled by delete, so that should be automatic
	return 0;
}