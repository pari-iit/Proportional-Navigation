#ifndef FILTER_H
#define FILTER_H
#include <vector>
#include <map>
#include "Dynamics.h"
#include "Measurement.h"

//Generic Filtering Stub
class Filter{
    public:
    virtual State update(const State& st, const Measurement& m) = 0;
};

//Define KF, EKF etc










#endif