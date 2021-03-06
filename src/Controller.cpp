/*******************************************************************************
 * Copyright (C) 2020 Parikshit Dutta
 *
 * This file is a part of target interception using Proportional Navigation (PN)
 * Algorithm
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Parikshit Dutta
#include "Controller.h"
#include <cassert>
#include <exception>

Control ConstantControl::generateControlOneStep(const State& st){
        
    try{
        assert(_K.cols() == st.x().size());
    } catch (const std::exception& e){
        printf("Mismatch in Gain and state vector sizes. %s\n", e.what());
        return {0,Eigen::VectorXf::Zero(1,1)};
    }
    return {st.t(),_K*st.x()};
}

void ConstantControl::setGain(const std::any& K){
    try{
        _K = std::any_cast<Eigen::MatrixXf>(K);
    }
    catch(const std::bad_any_cast& e) {
        printf("%s\n",e.what());
    }
}

void ConstantControl::generateControl(const std::vector<State>& st){
    try{
        assert(_K.rows() != 0 && _K.cols() != 0);
    } catch(const std::exception& e){
        printf("Gain is zero. Fix gain. %s\n",e.what());
    }
    
    int nt = st.size();
    for( auto s:st){
        _U_t.emplace_back(generateControlOneStep(s));
    }
}




void ProNav::setGain(const std::any& N){
    try{
        _N = std::any_cast<double>(N);
    }
    catch(const std::bad_any_cast& e) {
        printf("%s\n",e.what());
    }
}


//st is relative state vector. Own state - relative state of the target
void ProNav::generateControl(const std::vector<State>& st){
    std::unique_lock<std::mutex> lck(_mut);
    _U_t.clear();
    for (auto s: st ){
        assert( s.x().size() == 2*_N_STATE);// Point mass model
        Eigen::VectorXf x = s.x();

        Eigen::Vector3f r,v;

        
        if (_N_STATE == 2){
            r << x.head(_N_STATE),Eigen::VectorXf::Zero(1);
            v << x.tail(_N_STATE),Eigen::VectorXf::Zero(1);        
        }
        else{
            r << x.head(_N_STATE); v << x.tail(_N_STATE);
        }
            
        

        Eigen::Vector3f omega = ( r.cross(v))/( r.dot(r) );        
        double mult = (-1.0*_N* (v.norm())/(r.norm()) );
        Eigen::VectorXf a  = r.cross(omega);
        a = mult* a;        

        Control u(s.t(),a.head(_N_CONTROL));    

        _U_t.emplace_back(u);    
    }
    
}