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
  std::string _ifile;
  
  
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

  

  std::unique_ptr<Controller> _c;
  std::unique_ptr<Filter> _f;
  std::unique_ptr<MeasurementModel> __m;
  std::unique_ptr<Simulator> _sim;  
  void readFile();
public:
  SimManager(const std::string& ifile):_ifile(ifile){}
  bool runSimulation(const std::string& resDir);
  void generateMeasurement(const std::string measDir);

};


#endif