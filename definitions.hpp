#include <array>

#ifndef DEFINICJE_HPP
#define DEFINICJE_HPP

#define PLANT_TIME_CONSTANT 627.0
#define DESIRED_TIME_CONSTANT 300
#define SETPOINT 40.0
#define INITIAL_CONDITIONS 0
#define SIMULATION_TIME 50
#define SAMPLING_TIME 125.0
#define PLANT_MODEL_A 0.8187
#define PLANT_MODEL_B 0.1088
#define PLANT_TRANSPORT_DELAY 77.0
#define PREDICTION_HORIZON 1
#define MAX_INPUT 100
#define MIN_INPUT 0

using namespace std;

const std::array<unsigned, SIMULATION_TIME + 1> TIME_SAMPLES = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                                                            11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                                                            21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
                                                            31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
                                                            41, 42, 43, 44, 45, 46, 47, 48, 49, 50};



#endif