/**
 * @file AppUpDownSphere.cpp
 * @brief App to rotate a full tensegrity structure in a water like fluid.
 * @author Victor Faraut
 * $Id$
 */


// This application
#include "yamlbuilder/TensegrityModel.h"
//include controller
#include "buoyancySimulator.h"
#include "upDownBCUController.h"
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
void createAndAttachedBuoyancySimulator(TensegrityModel* const myModel);
void createAndAttachedRotBCUController(TensegrityModel* const myModel);

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
    tgWorld world(config, ground); 

    // Create the view
    const double timestep_physics = 0.0001; // seconds
    const double timestep_graphics = 1.f/60.f; // seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
    // Create the simulation
    tgSimulation* simulation = new tgSimulation(view);

    createAndAttachedBuoyancySimulator(myModel);
    createAndAttachedRotBCUController(myModel);
        

    simulation->addModel(myModel);

    simulation->run();

    delete(simulation);
    delete(ground);
    delete(myModel);

    simulation = nullptr;
    ground= nullptr;

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

      modelPath = "src/dev/vfaraut/yamlFiles/SphereYAML/BCU_sphere.yaml";
    }
    else
    {
      modelPath = userModelPath;
    }
    return new TensegrityModel(modelPath,false);
}


/**
 * Creats the buoyancy simulator for rod and attached it to a model
 * @param[in] Model that you want your buoyancy simulator to be attached to
 */
void createAndAttachedBuoyancySimulator(TensegrityModel* const myModel){

  float waterHeight = 1000.0;
  float waterDensity = 1.025;
  std::vector<std::string> tagsToControl;

  tagsToControl.push_back("s00");

  tgObserver<TensegrityModel>* const controller = 
    new buoyancySimulator(waterHeight, waterDensity, tagsToControl);

  myModel->attach(controller);

  return;
}

/**
 * Creats the buoyancy simulator for rod and attached it to a model
 * @param[in] Model that you want your buoyancy simulator to be attached to
 */
void createAndAttachedRotBCUController(TensegrityModel* const myModel){
  std::vector<std::string> tagsToControl;

  tagsToControl.push_back("s00");

  tgObserver<TensegrityModel>* const controller = 
    new upDownBCUController(tagsToControl);

  myModel->attach(controller);

  return;
}
