#include "Simulator.h"
#include <cassert>
#include <fstream>

std::vector<State> Simulator::Simulate(State& st, const double& ti, const double& tf){
    try{
        assert( fabs(st.t() - ti ) <1e-5);
    }
    catch (std::exception& e){
        throw("initial state and starting time do not match. Assuming initial starting time.\n");
    }
    double t = ti; 
    std::vector<State> s_hist;    
    while(t <tf){
        std::unique_lock<std::mutex> ulock(_mut);
        s_hist.push_back(st);
        _dyn->propagate(st,_dt);
        t = t+_dt;
        ulock.unlock();
    }
    
    return s_hist;
}
