#include <math.h>
#include "models.hpp"

using namespace std;



void ReferenceModel::referenceModelInit(double desiredTimeConstant, double samplingTime, double initConditions)
{
    this->initialCondition = initConditions;

    if(desiredTimeConstant > 0 && samplingTime > 0){
        this->lambda = exp(-samplingTime / desiredTimeConstant);
    }
}

void ReferenceModel::determiningOutputTrajectory(unsigned setpoint, std::array <unsigned, TIME_SAMPLES.size()> timeSamples){
    for(unsigned i = 0; i < timeSamples.size(); i++){
        this->referenceModelOutput[i] = setpoint - ((setpoint - this->initialCondition) * pow(this->lambda, (i)));
    }
}

void FirstOrderPlantModel::modelInit(double a, double b, double delay){
    this->am = a;
    this->bm = b;
    this->delayInSamples = (unsigned)ceil(delay / SAMPLING_TIME);
}

void FirstOrderPlantModel::determiningOutput(unsigned TimeSample){
    if(TimeSample == 0){
        this->outputSignalValues[TimeSample] = this->getInitialConditions();
    }
    else{
        this->outputSignalValues[TimeSample] =  this->am * this->outputSignalValues[TimeSample - 1] +
                                                this->bm * this->inputSignalValues[TimeSample];
    }
}

void FirstOrderPlantModel::setInput(double currentInput, unsigned currentTimeSample){
    if(currentTimeSample < TIME_SAMPLES.size()){
        this->inputSignalValues[currentTimeSample] = currentInput;
    }
}

unsigned FirstOrderPlantModel::getDelayInSamples(){
    return this->delayInSamples;
}

double FirstOrderPlantModel::getInitialConditions(){
    return this->initConditions;
}

double FirstOrderPlantModel::getInputSignal(unsigned currentTimeSample){
    return this->inputSignalValues[currentTimeSample];
}

double FirstOrderPlantModel::getOutput(unsigned TimeSample){
    return this->outputSignalValues[TimeSample];
}

double *FirstOrderPlantModel::getOutputSignal()
{
    return this->outputSignalValues;
}

double *FirstOrderPlantModel::getDelayedOutputSignal()
{
    return this->delayedOutputSignalValues;
}

void FirstOrderPlantModel::includeDelayToOutput(double *outputSignalVector, double *delayedOutputSignalVector, unsigned delaySamples)
{
    for (unsigned i = 0; i < TIME_SAMPLES.size(); i++) {
        if (i + delaySamples < TIME_SAMPLES.size()) {
            delayedOutputSignalVector[i + delaySamples] = outputSignalVector[i];
        }
    }
}

double ReferenceModel::getLambda(){
    return this->lambda;
}

double *ReferenceModel::getReferenceOutput(){
    return this->referenceModelOutput;
}
