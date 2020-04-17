/*******************************************************************************
 * Copyright (C) 2020 Parikshit Dutta
 *
 * This file is a part of target interception using Proportional Navigation (PN)
 * Algorithm
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Parikshit Dutta
#include <iostream>
#include "SimManager.h"



void printUsage(const std::string& fname){
    std::cout << "Usage: ./simulator <inputfile>\n"
            <<" Using inputfile = " << fname << std::endl;
    
}

int main(int argc, char** argv) {
    std::string in_file = "../Data/inputfile_3d.txt";
    if(argc < 2){
        printUsage(in_file);
    }
    else{
        in_file = std::string(argv[1]);        
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