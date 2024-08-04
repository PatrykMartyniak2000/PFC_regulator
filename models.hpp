#ifndef MODELS_HPP_
#define MODELS_HPP_

#include "definitions.hpp"

class ReferenceModel{
    protected:
        double initialCondition = INITIAL_CONDITIONS;
        double lambda;
        double referenceModelOutput[TIME_SAMPLES.size()] = {0};
    public:
        double getLambda();
        double *getReferenceOutput();

        // this function is determining discrete pole of the reference model
        void referenceModelInit(double desiredTimeConstant, double samplingTime, double initConditions);
        // this function is determining whole reference trajectory
        void determiningOutputTrajectory(unsigned setpoint, std::array <unsigned, TIME_SAMPLES.size()> timeSamples);
};

class FirstOrderPlantModel{
    protected:
        double am;
        double bm;
        unsigned delayInSamples = 0;
        double initConditions = INITIAL_CONDITIONS;
        double inputSignalValues [TIME_SAMPLES.size()] = {0};
        double outputSignalValues [TIME_SAMPLES.size()] = {0};
        double delayedOutputSignalValues [TIME_SAMPLES.size()] = {0};

    public:
        void modelInit(double am, double bm, double transportDelay);
        void determiningOutput(unsigned currentTimeSample);
        void setInput(double currentInput, unsigned currentTimeSample);
        unsigned getDelayInSamples();
        double getInitialConditions();
        double getInputSignal(unsigned currentTimeSample);
        double getOutput(unsigned currentTimeSample);
        double *getOutputSignal();
        double *getDelayedOutputSignal();
        void includeDelayToOutput(double *outputSignalVector, double *delayedOutputSignalVector, unsigned delaySamples);
};

#endif