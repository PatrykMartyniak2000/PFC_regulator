#ifndef CONTROL_LAW_HPP_
#define CONTROL_LAW_HPP_

#include "definitions.hpp"

class ControlLow
{
protected:
    double error = SETPOINT - INITIAL_CONDITIONS;
    double inputSignal[TIME_SAMPLES.size()] = {0};
    
public:
    void determiningError(double currentPlantModelOutput);
    void determiningInputSignal(double lambda, unsigned predictionHorizon, double referenceModelOutput, unsigned currentTimeSample);
    double getInput(unsigned currentTimeSample);
};



#endif