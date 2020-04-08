#include "Simulator.h"
#include <cassert>
#include <fstream>


void Simulator::SimStep(State& st,const std::vector<Control>& u){
    if( u.size() == 1){
        _dyn->propagate(st,u[0],_dt);
    }
    else{
        auto it = std::find_if(u.begin(), u.end(), [&st](Control c){
            return (st.t() < c.time());
        });
        
        Control c(0,Eigen::VectorXf::Zero(_N_CONTROL));            
        
        if (it > u.begin()){
            c = u[it-u.begin()-1];
        }

        _dyn->propagate(st,c,_dt);
    }
        
        
}

std::vector<State> Simulator::Simulate(State&& st, const double&& ti, const double&& tf){
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


std::vector<State> Simulator::SimulateControl(State&& st, const double&& ti, const double&& tf,
const std::vector<Control>&& U){
    try{
        assert( U.size() > 0);
    }
    catch(std::exception& e){
        printf("Input vector empty. Using autonomous system.\n");
        double cpyti = ti,cpytf = tf;
        return Simulate(std::move(st),std::move(cpyti),std::move(cpytf));
    }

    try{
        assert( fabs(st.t() - ti ) <1e-5);
    }
    catch (std::exception& e){
        throw("initial state and starting time do not match. Assuming initial starting time.\n");
    }    
    double t = ti; 
    std::vector<State> s_hist; 
    int ccnt = 0; Control u = U[ccnt];
    if (u.time() > t){
        ccnt--;
        u = {t,Eigen::VectorXf::Zero(_N_CONTROL)};
    }
    while(t <tf){
        std::unique_lock<std::mutex> ulock(_mut);        
        s_hist.push_back(st);        
        _dyn->propagate(st,u,_dt);
        t = t+_dt;
        if (ccnt < U.size()-1 && t > U[ccnt+1].time()) {
            u = U[++ccnt];
        }        
        ulock.unlock();
    }
    
    return s_hist;
}

