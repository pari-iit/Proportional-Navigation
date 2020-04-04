#ifndef SIMMANAGER_H
#define SIMMANAGER_H

#include <vector>
#include <memory>
#include <thread>
#include <string>
#include "Controller.h"
#include "Filter.h"
#include "Measurement.h"
#include "Simulator.h"


class SimManager{
    
  //generating measurement
  std::vector<State> _ics;

  //Dynamics
  Eigen::MatrixXf _F;
  Eigen::MatrixXf _Q;
  Eigen::MatrixXf _B;

  //Measurement
  Eigen::MatrixXf _R;

  //Proportional Navigation
  double _N; 

  //Simulator 
  //For simplicity filter and simulator run at the same speed
  double _dt;

  

  //pointers to individual programs
  std::unique_ptr<Controller> _c;
  //Using Filter here instead of KalmanFilter gives valgrind error. Dont know why. 
  std::unique_ptr<KalmanFilter> _f;
  std::unique_ptr<Simulator> _sim;  

  //reading input file and constructing the inputs
  void readFile(const std::string& ifile);
public:
  SimManager(const std::string& ifile);
  bool runSimulation(const std::string& resDir);
  void generateMeasurement(const std::string measDir);

};


#endif