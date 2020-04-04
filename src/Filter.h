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
    virtual void update(State& st, const Measurement& m, const State& sref) = 0;
};

//Define KF, EKF etc

class KalmanFilter:public Filter{    
    std::shared_ptr<Dynamics> _dyn;
    std::unique_ptr<MeasurementModel> _meas;//Make sure measurement model is only used by a the filtering module.
    void updateStep( State& s, const Measurement& m, const State& sref); 
    void propagateStep( State& st);
public:
    KalmanFilter(std::shared_ptr<Dynamics> dyn,  std::unique_ptr<MeasurementModel> meas):_dyn(std::move(dyn)),_meas(std::move(meas)){};
    void update(State& st, const Measurement& m, const State& sref);

};










#endif