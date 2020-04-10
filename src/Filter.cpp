#include "Filter.h"
#include <cassert>
#include <algorithm>
#include <functional>

using Eigen::MatrixXf;
using Eigen::VectorXf;

void KalmanFilter::propagateStep(State& st, const double& dt){        
    _dyn->propagate(st,dt);
}

void KalmanFilter::updateStep( State& s, const Measurement& m, const State& sref){

    VectorXf x_rel = s.x()-sref.x();
    State st(s.t(),std::move(x_rel),s.P() );
    VectorXf em = _meas->estimateMeasurement(st);

    //Calculate innovation
    assert( em.size() ==  m.Y().size());
    VectorXf I = m.Y() - em;
    

    // Calculate Innovation covariance;
    MatrixXf S = ( (_meas->Jacobian(st)) * st.P() * (_meas->Jacobian(st)).transpose() ) + m.R();
    MatrixXf K = st.P() * ( (_meas->Jacobian(st)).transpose() ) * (S.inverse());


    st.setX(st.x() + K*I); 
    st.setP( ( MatrixXf::Identity( st.P().rows(),st.P().cols() ) - K*_meas->Jacobian(st) )*st.P()  );

    s.setX(st.x()+sref.x());
    s.setP(st.P());
} 

void KalmanFilter::update(State& st, const Measurement& m, const State& sref, const double& dt){
    propagateStep(st, dt);
    updateStep(st,m,sref);
}