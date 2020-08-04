/**
 * @file AppRodTest.cpp
 * @brief Basic Testing of YAML and SB model building with functions. Simple 
 * controller as well
 * @author Victor Faraut
 * $Id$
 */


// This application
#include "yamlbuilder/TensegrityModel.h"
//include controller
#include "buoyancySimulator.h"
#include "simpleBCUController.h"
#include "rotBCUController.h"
#include "StepwiseController.h"
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
void createAndAttachedBCUController(TensegrityModel* const myModel);
void createAndAttachedRotBCUController(TensegrityModel* const myModel);
void createAndAttachedStepwiseController(TensegrityModel* const myModel);

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

    // create the view
    const double timestep_physics = 0.0001; // seconds
    //const double timestep_physics = 0.001;
    const double timestep_graphics = 1.f/60.f; // seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
    // create the simulation
    tgSimulation* simulation = new tgSimulation(view);

    createAndAttachedBuoyancySimulator(myModel);
    //createAndAttachedBCUController(myModel);
    createAndAttachedRotBCUController(myModel);
    //createAndAttachedStepwiseController(myModel);
        

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

      modelPath = "src/dev/vfaraut/yamlFiles/TurtleYAML/BCU_sphere_rod.yaml";
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

  tagsToControl.push_back("alu_rod_XZ_0");
  tagsToControl.push_back("alu_rod_XZ_1");
  tagsToControl.push_back("alu_rod_YX_0");
  tagsToControl.push_back("alu_rod_YX_1");
  tagsToControl.push_back("alu_rod_ZY_0");
  tagsToControl.push_back("alu_rod_ZY_1");

  tgObserver<TensegrityModel>* const controller = 
    new buoyancySimulator(waterHeight, waterDensity, tagsToControl);

  myModel->attach(controller);

  return;
}

/**
 * Creats the buoyancy simulator for rod and attached it to a model
 * @param[in] Model that you want your buoyancy simulator to be attached to
 */
void createAndAttachedBCUController(TensegrityModel* const myModel){
  std::vector<std::string> tagsToControl;

  // tagsToControl.push_back("alu_rod_XZ_0");
  // tagsToControl.push_back("alu_rod_XZ_1");
  tagsToControl.push_back("alu_rod_YX_0");
  // tagsToControl.push_back("alu_rod_YX_1");
  //tagsToControl.push_back("alu_rod_ZY_0");
  //tagsToControl.push_back("alu_rod_ZY_1");

  tgObserver<TensegrityModel>* const controller = 
    new simpleBCUController(tagsToControl);

  myModel->attach(controller);

  return;
}

/**
 * Creats the buoyancy simulator for rod and attached it to a model
 * @param[in] Model that you want your buoyancy simulator to be attached to
 */
void createAndAttachedRotBCUController(TensegrityModel* const myModel){
  std::vector<std::string> tagsToControl;

  tagsToControl.push_back("alu_rod_XZ_0");
  tagsToControl.push_back("alu_rod_XZ_1");
  tagsToControl.push_back("alu_rod_YX_0");
  tagsToControl.push_back("alu_rod_YX_1");
  tagsToControl.push_back("alu_rod_ZY_0");
  tagsToControl.push_back("alu_rod_ZY_1");

  tgObserver<TensegrityModel>* const controller = 
    new rotBCUController(tagsToControl);

  myModel->attach(controller);

  return;
}

void createAndAttachedStepwiseController(TensegrityModel* const myModel){
  // Attach a controller to the model, if desired.
    // This is a controller that interacts with a generic TensegrityModel as
    // built by the TensegrityModel file

    // Parameters for the SingleCableController or StewiseController are specified in that .h file,
    // repeated here:
    double startTime = 5.0;
    double stepTime = 7.0;
    double minLength = 0.1;
    double maxLength = 100;
    double rate = 0.5;
    std::vector<std::string> tagsToControl;
    //tagsToControl.push_back("m18"); // Tag located in SBv2_yaml_files/SBv2_model.yaml line: 28

    tagsToControl.push_back("m10");
    tagsToControl.push_back("m01");
    tagsToControl.push_back("m21");
    tagsToControl.push_back("m15");
    tagsToControl.push_back("m08");
    tagsToControl.push_back("m20");

    // tagsToControl.push_back("m01");
    // tagsToControl.push_back("m02");
    // tagsToControl.push_back("m03");
    // tagsToControl.push_back("m04");
    // tagsToControl.push_back("m05");
    // tagsToControl.push_back("m06");
    // tagsToControl.push_back("m07");
    // tagsToControl.push_back("m08");
    // tagsToControl.push_back("m09");
    // tagsToControl.push_back("m10");
    // tagsToControl.push_back("m11");
    // tagsToControl.push_back("m12");
    // tagsToControl.push_back("m13");
    // tagsToControl.push_back("m14");
    // tagsToControl.push_back("m15");
    // tagsToControl.push_back("m16");
    // tagsToControl.push_back("m17");
    // tagsToControl.push_back("m18");
    // tagsToControl.push_back("m19");
    // tagsToControl.push_back("m20");
    // tagsToControl.push_back("m21");
    // tagsToControl.push_back("m22");
    // tagsToControl.push_back("m23");
    // tagsToControl.push_back("m24");

    // Call the constructor for the controller
    StepwiseController* const controller =
      new StepwiseController(startTime, stepTime, minLength, maxLength, rate, tagsToControl);

    // Attach the controller to the model. Must happen before running the
    // simulation.
    myModel->attach(controller);
}
