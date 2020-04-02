#include <iostream>
#include "Dynamics.h"
#include "Controller.h"
#include "Filter.h"
#include "Measurement.h"
#include "Simulator.h"

int main() {

    std::unique_ptr<Controller> c(new ProNav(3));
    std::cout << "Hello World!" << "\n";

    //First simulate the robot and store it

    //Then use this measurement to guide the other robot
    //1. obtain measurement
    //2. Filter it
    //3. Generate control law.
    //4. Use control law to move
    return 0;
}