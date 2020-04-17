/*******************************************************************************
 * Copyright (C) 2020 Parikshit Dutta
 *
 * This file is a part of target interception using Proportional Navigation (PN)
 * Algorithm
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Parikshit Dutta
#include "Dynamics.h"
#include <cassert>

void LinearDynamics::propagate(State& s, const Control& u, const double& dt){
    
    if (dt <= 0){ // discrete-discrete
        s.setX(_F*(s.x()) + _B*(u.control()));
        s.setP(_F*(s.P())*(_F.transpose()) + _Q );
        s.setT(s.t()+1);
    } 
    else{//continuous-discrete
        s.setX(_F*(s.x())*dt + _B*(u.control())*dt + s.x() );
        s.setP(_F*(s.P())*dt + (s.P())*(_F.transpose())*dt + _Q*dt +s.P() );
        s.setT(s.t()+dt);
    }
}



void LinearDynamics::propagate(State& s, const double& dt){
    
    if (dt <= 0){ // discrete-discrete time
        s.setX(_F*(s.x()) );
        s.setP(_F*(s.P())*(_F.transpose()) + _Q );
        s.setT(s.t()+1);
    } 
    else{//continuous-discrete Kalman filter
        s.setX(_F*(s.x())*dt + s.x() );
        s.setP(_F*(s.P())*dt + (s.P())*(_F.transpose())*dt + _Q*dt + s.P() );
        s.setT(s.t()+dt);
    }
}


