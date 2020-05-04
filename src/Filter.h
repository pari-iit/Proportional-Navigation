/*******************************************************************************
 * Copyright (C) 2020 Parikshit Dutta
 *
 * This file is a part of target interception using Proportional Navigation (PN)
 * Algorithm
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Parikshit Dutta
#ifndef FILTER_H
#define FILTER_H
#include <vector>
#include <map>
#include <memory>
#include "Dynamics.h"
#include "Measurement.h"    

//Generic Filtering Stub
class MeasurementModel;
class Filter{
    public:
    virtual ~Filter() {};
    //Rule of five
    Filter () = default;

    //Rule of five            
    Filter(const Filter& that) = default;
    Filter& operator=(const Filter& that) = default;
    Filter(Filter&& that) = default;
    Filter& operator=(Filter&& that) = default;

    virtual void update(State& st, const Measurement& m, const State& sref, const double& dt) = 0;
};

//Define KF, EKF etc

class KalmanFilter:public Filter{    
    std::shared_ptr<Dynamics> _dyn;
    std::unique_ptr<MeasurementModel> _meas;//Make sure measurement model is only used by a the filtering module.
    void updateStep( State& s, const Measurement& m, const State& sref); 
    void propagateStep( State& st, const double& dt);
public:
    KalmanFilter(std::shared_ptr<Dynamics> dyn,  std::unique_ptr<MeasurementModel> meas):_dyn(std::move(dyn)),_meas(std::move(meas)){};
    void update(State& st, const Measurement& m, const State& sref, const double& dt);

};










#endif