/**
 * @file AppSimpleTest.cpp
 * @brief Basic Testing of YAML and SB model building. Should be
 * depricated once the offical model is done
 * $Id$
 */

// This application
#include "yamlbuilder/TensegrityModel.h"

//include controller
//#include "BasicController.h"
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

TensegrityModel* createModel(char const *argv[]);
tgSimulation createSimulation(void);

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @param[in] argv argv[1] is the path of the YAML encoded structure
 * @return 0
 */

int main(int argc, char const *argv[])
{
  
    TensegrityModel* const myModel = createModel(argv);

    tgSimulation simulation = createSimulation();
    //createController();



    return 0;
}


/**
 * Function that creates the model for the simulation based on the default seting or user define path
 * @return the TensegrityModel object
 */
TensegrityModel* createModel(char const *argv[]){

    std::string modelName = "";
    if (argv[1] == NULL)
    {
      std::cout << "No arguments passed in to the application. Default model was used. If you want to use a custom model, you need to specify it"
	    << std::endl;

      modelName = "src/dev/jbruce/SBv2/SBv2_yaml_files/SBv2_model_payload.yaml";

    }
    else
    {
      modelName = argv[1];
      std::invalid_argument(std::string(argv[1]));
    }
    
    return new TensegrityModel(modelName,false);
    
}


/**
 * Function that creates the world, view and returns the linked tgSimulation.
 * @return the simulation object to be run elsewhere
 */
tgSimulation createSimulation(void){
    const double yaw        = 0.0;
    const double pitch      = 0.0;
    const double roll       = 0.0;

    //Creats a config for the new ground. Can be un-nececarry as the angle are zero
    //and the default angles too.
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    tgBoxGround* ground = new tgBoxGround();

    const tgWorld::Config config(98.1);
    tgWorld world(config, ground);

    // create the view
    const double timestep_physics = 0.0001; // seconds
    //const double timestep_physics = 0.001;
    const double timestep_graphics = 1.f/60.f; // seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // create the simulation
    tgSimulation simulation(view);
     
    return simulation;
}

//BasicController* 
