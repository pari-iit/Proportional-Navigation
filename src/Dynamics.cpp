#include "Dynamics.h"
#include <cassert>

void LinearDynamics::propagate(State& s, const double& dt){
    
    if (dt <= 0){ // discrete-discrete time
        s.setX(_F*(s.x()) );
        s.setP(_F*(s.P())*(_F.transpose()) + _Q );
        s.setT(s.t()+1);
    } 
    else{//continuous-discrete Kalman filter
        s.setX(_F*(s.x())*dt + s.x() );
        s.setP(_F*(s.P()) + (s.P())*(_F.transpose()) + _Q );
        s.setT(s.t()+1);
    }
}


void NonlinearDynamics::propagate(State& s, const double& dt){
}



