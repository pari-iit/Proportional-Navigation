#ifndef SIMMANAGER_H
#define SIMMANAGER_H

#include <vector>
#include <memory>
#include <thread>
#include <string>
#include <unordered_map>
#include "Controller.h"
#include "Filter.h"
#include "Measurement.h"
#include "Simulator.h"

class SimManager{
  std::mutex _mut;

  //Target trajectory
  std::vector<double> _tf;
  std::vector<State> _ics;
  std::vector<std::vector<Control> > _u;

  //Interceptor trajectory
  std::vector<State> _iloc;
  std::vector<Control>  _iu;

  //Interceptor Trajectory history
  std::vector<std::vector<State> > _itraj;

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
  //Used to generate spurious measurement from target states. This is different from the Kalman filter measurement.
  std::unique_ptr<MeasurementModel> _m;
  //Using Filter here instead of KalmanFilter gives valgrind error. Dont know why. 
  std::unique_ptr<KalmanFilter> _f;
  //Multiple threads can use the same pointer. 
  std::shared_ptr<Simulator> _sim;  

  //reading input file and constructing the inputs
  State readState(const std::vector<std::string>& tokens);
  std::vector<Control> readControlSequence(const std::vector<std::string>& tokens);
  void readFile(const std::string& ifile);
  void writeFile(const std::string& ofile, const std::vector<State>& st);

  //Simulator helpers
  std::unordered_map<int,int> mapTargetToInterceptor();
  std::vector<State> simulateProNav(const int&& t_id, const int&& i_id);
  
public:
  SimManager(const std::string& ifile);
  void runSimulation(const std::string& resDir);
  void generateMeasurement(const std::string measDir);

};


#endif