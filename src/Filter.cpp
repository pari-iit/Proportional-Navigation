#include "Filter.h"
#include <cassert>
#include <algorithm>
#include <functional>

void KalmanFilter::propagateStep(State& st ){    
    double dt = -1;
    _dyn->propagate(st,dt);
}

void KalmanFilter::updateStep(const Measurement& m,State& st){
    std::vector<double> em = _meas->estimateMeasurement(st);

    //Calculate innovation
    assert( em.size() ==  m.Y().size());
    std::vector<double> I;
    std::transform(m.Y().begin(),m.Y().end(),em.begin(),std::back_inserter(I), std::minus<int>());

    // Calculate Innovation covariance;
    std::vector<std::vector<double> > S;
    

} 
