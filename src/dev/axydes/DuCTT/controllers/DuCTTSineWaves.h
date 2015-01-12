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

#ifndef DUCTT_SINE_WAVES_H
#define DUCTT_SINE_WAVES_H

/**
 * @file DuCTTSineWaves.h
 * @brief Contains the definition of the class DuCTTSineWaves.
 * @author Alexander Xydes
 * $Id$
 */

// This library
#include "core/tgObserver.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"

// The C++ Standard Library
#include <vector>

// Forward declarations
class tgBasicActuator;
class tgPrismatic;
class tgTouchSensorSphereModel;
class DuCTTRobotModel;
class tgImpedanceController;

class DuCTTSineWaves : public tgObserver<DuCTTRobotModel>
{
public:

    DuCTTSineWaves(double targetDist = -1);
    
    virtual void onStep(DuCTTRobotModel& subject, double dt);
    
    void applySineWave(tgPrismatic* prism, bool shouldPause, bool shouldUnPause, double dt, int phase=0);
    bool shouldPause(std::vector<tgTouchSensorSphereModel*> touchSensors);

    /**
     * Applies the impedance controllers using a velocity setpoint of 0.
     * Called during this classes onStep function.
     * @param[in] stringList a std::vector of strings taken from the
     * subject's MuscleMap
     * @param[in] dt - a timestep. Must be positive.
     */
    void applyImpedanceControlInside(const std::vector<tgBasicActuator*> stringList,
                                    double dt);
    /**
     * Applies the impedance controllers using a velocity setpoint determined.
     * by the phase parameter and
     * Called during this classes onStep function.
     * @param[in] stringList a std::vector of strings taken from the
     * subject's MuscleMap
     * @param[in] dt - a timestep. Must be positive.
     * @param[in] phase - reads the index out of the phaseOffsets vector
     */
    void applyImpedanceControlOutside(const std::vector<tgBasicActuator*> stringList,
                                    double dt,
                                    std::size_t phase);

private:
    tgImpedanceController* in_controller;
    tgImpedanceController* out_controller;

    double simTime;
    double cycle;
    double target;

    std::vector<double> phaseOffsets;
    const double offsetLength;
    const double cpgAmplitude;
    const double cpgFrequency;
    const double bodyWaves;

    double cyclePrism;
    double targetPrism;
    const double offsetLengthPrism;
    const double cpgAmplitudePrism;
    const double cpgFrequencyPrism;
    const double bodyWavesPrism;

    bool recordedStartCOM;
    btVector3 startCOM;

    const double insideLength;

    double targetDist;
    bool move;
};

#endif // PRETENSION_CONTROLLER_H