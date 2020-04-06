#include <iostream>
#include "SimManager.h"

int main(int argc, char** argv) {
    std::string in_file = "../Data/inputfile.txt";
    if(argc <=1){
        printf("No arguments.Using default file\n");
    }
    else{        
        in_file = std::string(argv[0]);
    }
    
    std::unique_ptr<SimManager> sim(new SimManager(in_file) );
    sim->generateMeasurement("../Data");
    std::cout << "Hello World!" << "\n";

    //First simulate the robot and store it

    //Then use this measurement to guide the other robot
    //1. obtain measurement
    //2. Filter it
    //3. Generate control law.
    //4. Use control law to move
    return 0;
}