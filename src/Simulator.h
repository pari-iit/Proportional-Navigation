#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include "Dynamics.h"

class Dynamics;
class Simulator{
    int const _N_CONTROL;
    double _dt;
    std::shared_ptr<Dynamics> _dyn;
    std::mutex _mut;    
  public:
    Simulator(const int& nc,const double& dt,std::shared_ptr<Dynamics> dyn):_N_CONTROL(nc),_dt(dt),_dyn(std::move(dyn)){}  
    std::vector<State> Simulate(State&& st,const double&& ti, const double&& tf );
    std::vector<State> SimulateControl(State&& st,const double&& ti, const double&& tf, const std::vector<Control>&& U );
    void SimStep(State& st,const std::vector<Control>& u);
};


#endif
