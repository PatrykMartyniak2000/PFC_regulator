#include "control_law.hpp"
#include "definitions.hpp"
#include "models.hpp"
#include <math.h>

void ControlLow::determiningError(double nextPlantModelOutput){
    this->error = SETPOINT - nextPlantModelOutput;
}

void ControlLow::determiningInputSignal(double lambda, unsigned predictionHorizon, double plantModelCurrentOutput, unsigned currentTimeSample){
    this->inputSignal[currentTimeSample] = (1 - pow(lambda, (double)predictionHorizon)) * this->error;
    this->inputSignal[currentTimeSample] += (1 - pow(PLANT_MODEL_A, (double)predictionHorizon)) * plantModelCurrentOutput;
    this->inputSignal[currentTimeSample] /= (PLANT_MODEL_B * (1 - pow(PLANT_MODEL_A, (double)predictionHorizon)));
    this->inputSignal[currentTimeSample] *= (1 - PLANT_MODEL_A);

    // constrains handling
    this->inputSignal[currentTimeSample] = (this->inputSignal[currentTimeSample] > MAX_INPUT) ? MAX_INPUT : this->inputSignal[currentTimeSample];
    this->inputSignal[currentTimeSample] = (this->inputSignal[currentTimeSample] < MIN_INPUT) ? MIN_INPUT : this->inputSignal[currentTimeSample];
}

double ControlLow::getInput(unsigned TimeSample)
{
    return this->inputSignal[TimeSample];
}
