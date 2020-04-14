#include <iostream>
#include "SimManager.h"



void printUsage(const std::string& fname){
    std::cout << "Usage ./simulator <inputfile> <dof>\n"
            <<" Using dof = 3, inputfile = " << fname << std::endl;
    
}

int main(int argc, char** argv) {
    std::string in_file = "../Data/inputfile_3d.txt";
    int dof = 3;
    if(argc < 2){
        printUsage(in_file);
    }
    else if (argc == 2){              
        in_file = std::string(argv[1]);                
        printUsage(in_file);
    }
    else{
        in_file = std::string(argv[1]);        
        dof = std::atoi(argv[2]);
    }
    
    std::unique_ptr<SimManager> sim(new SimManager(in_file) );
    sim->runSimulation("../Data");
    std::cout << "Simulation completed.\n" 
    << "Results in ../Data. \n"
    <<"To view results, please change directory to ../Data and run python postprocessor.py.\n" ;


    //First simulate the robot and store it

    //Then use this measurement to guide the other robot
    //1. obtain measurement
    //2. Filter it
    //3. Generate control law.
    //4. Use control law to move
    return 0;
}