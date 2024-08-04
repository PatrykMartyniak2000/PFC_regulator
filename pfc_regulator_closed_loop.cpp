#include <iostream>
#include <vector>
#include "models.hpp"
#include "control_law.hpp"
#include "definitions.hpp"


int main() {
    ReferenceModel reference_model;
    FirstOrderPlantModel plant_model;
    ControlLow control_law;

    reference_model.referenceModelInit(DESIRED_TIME_CONSTANT, SAMPLING_TIME, INITIAL_CONDITIONS);
    reference_model.determiningOutputTrajectory(SETPOINT, TIME_SAMPLES);
    plant_model.modelInit(PLANT_MODEL_A, PLANT_MODEL_B, PLANT_TRANSPORT_DELAY);
    
    for(unsigned i = 0; i < TIME_SAMPLES.size(); i++){
        control_law.determiningInputSignal(reference_model.getLambda(), PREDICTION_HORIZON, plant_model.getOutput(i - 1), (i));
        plant_model.setInput(control_law.getInput(i), i);
        plant_model.determiningOutput(i);
        control_law.determiningError(plant_model.getOutput(i));
    }

    plant_model.includeDelayToOutput(plant_model.getOutputSignal(), plant_model.getDelayedOutputSignal(),
                                                                        plant_model.getDelayInSamples());

    return 0;
}
