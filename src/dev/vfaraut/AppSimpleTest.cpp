/**
 * @file AppSimpleTest.cpp
 * @brief Basic Testing of YAML and SB model building with functions. Simple 
 * controller as well
 * @author Victor Faraut
 * $Id$
 */


// This application
#include "yamlbuilder/TensegrityModel.h"
//include controller
#include "SimpleController.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimulation.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgWorld.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <string>
#include <vector>

// Forward declarations
TensegrityModel* createModel(char const *userModelPath);
void createAndAttachedController(TensegrityModel* const myModel);

/**
 * Main function that creates model, simulation, and runs it 
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @param[in] argv argv[1] is the path of the YAML encoded structure
 * @return 0
 */

int main(int argc, char const *argv[])
{
    // Creates the model from the YAML file in the call of the App or with the 
    // default file
    TensegrityModel* const myModel = createModel(argv[1]);
    
    //tgSimulation* simulation = createSimulation();

    // Creates a new ground object
    tgBoxGround* ground = new tgBoxGround();

    const tgWorld::Config config(98.1);
    tgWorld world(config, ground); //degeu

    // create the view
    const double timestep_physics = 0.0001; // seconds
    //const double timestep_physics = 0.001;
    const double timestep_graphics = 1.f/60.f; // seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
    // create the simulation
    tgSimulation* simulation = new tgSimulation(view);

    createAndAttachedController(myModel);

    simulation->addModel(myModel);

    simulation->run();

    delete(simulation);
    simulation = nullptr;
    return 0;
}


/**
 * Function that creates the tensegrity model with default path or user define 
 * path
 * @param[in] userModelPath is the model yaml path. Pass "NULL" is default
 * @return the TensegrityModel object
 */
TensegrityModel* createModel(char const *userModelPath){

    std::string modelPath = "";
    if (userModelPath == NULL)
    {
      std::cout << "No arguments passed in to the application. Default model "
      << "was used. If you want to use a custom model, you need to specify it"
	    << std::endl;

      modelPath = "src/dev/vfaraut/SBv2_yaml_files/SBv2_model_payload.yaml";

    }
    else
    {
      modelPath = userModelPath;
    }
    return new TensegrityModel(modelPath,false);
}

/**
 * Function that creates the world, view and returns the linked tgSimulation.
 ////////////////////////////////////////////////////////////////////////////
 Function doesn't work due to the implementation of view. View is using world
 directly as it is, and world is delete at the end of this function. The program
 end up with seg-fault
 ////////////////////////////////////////////////////////////////////////////
 * @return the simulation object to be run elsewhere
 
tgSimulation* createSimulation(void){
    const double yaw        = 0.0;
    const double pitch      = 0.0;
    const double roll       = 0.0;

    //Creats a config for the new ground. Can be un-nececarry as the angle are
    //zero and the default angles too.
    //const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    tgBoxGround* ground = new tgBoxGround();

    const tgWorld::Config config(98.1);
    tgWorld world(config, ground); //degeu

    // create the view
    const double timestep_physics = 0.0001; // seconds
    //const double timestep_physics = 0.001;
    const double timestep_graphics = 1.f/60.f; // seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
    // create the simulation
    tgSimulation* simulation = new tgSimulation(view);
    std::cout << "Debug1"<< std::endl;
    return simulation;
}
*/

/**
 * Function that creates the controller and attached it to the given model
 * @param[in] Model that you want your controller to be attached
 */
void createAndAttachedController(TensegrityModel* const myModel){

  double startTime = 5.0;
  double stepTime = 2.0;
  double minLength = 0.1;
  double rate = 3.5;
  std::vector<std::string> tagsToControl;
  //tagsToControl.push_back("m18"); // Tag located in SBv2_yaml_files/SBv2_model.yaml line: 28

  tagsToControl.push_back("m10");
  tagsToControl.push_back("m01");
  tagsToControl.push_back("m21");


  tagsToControl.push_back("m15");
  tagsToControl.push_back("m08");
  tagsToControl.push_back("m20");

  tgObserver<TensegrityModel>* const controller = new SimpleController(startTime, stepTime, minLength, rate, tagsToControl);

   myModel->attach(controller);

  return;
}
