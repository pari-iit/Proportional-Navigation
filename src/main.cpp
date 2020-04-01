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
    
    return 0;
}