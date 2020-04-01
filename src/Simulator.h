#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <vector>
#include <thread>
#include <memory>
#include <mutex>
#include "Dynamics.h"

class Dynamics;
class Simulator{
    double _dt;
    std::shared_ptr<Dynamics> _dyn;
    std::mutex _mut;    
  public:
    Simulator(const double& dt,std::shared_ptr<Dynamics> dyn, const State& ic):_dt(dt),_dyn(std::move(dyn)){}  
    std::vector<State> Simulate(State& st,const double& ti, const double& tf );
};


#endif
