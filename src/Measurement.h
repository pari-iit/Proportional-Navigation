#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <vector>
#include <stdio.h>
#include "Dynamics.h"

class Measurement{
    double _t;
    Eigen::VectorXf _y;
    Eigen::MatrixXf _R;

public:
    Measurement(const double& t, const Eigen::VectorXf& y, const Eigen::MatrixXf& R):_t(t), _y(y), _R(R){}

    //RULE OF FIVE
    ~Measurement(){
        printf("Destroying measurement for %f time.\n",_t);
    }

    Measurement(const Measurement& that):_t(that._t),_y(that._y),_R(that._R){}
    Measurement& operator=(const Measurement& that){
        _t = that._t,_y = that._y ,_R = that._R;
        return *this;
    }
    Measurement(Measurement&& that):_t(std::exchange(that._t,0.0)),_y(std::move(that._y)),_R(std::move(that._R)){}
    Measurement& operator=(Measurement&& that){
        _t = std::exchange(that._t,0.0),_y = std::move(that._y) ,_R = std::move(that._R);
        return *this;
    }

    double Time() const{ return _t;}
    Eigen::VectorXf Y() const{return _y;}
    Eigen::MatrixXf R() const{return _R;}


};

class MeasurementModel{
public:
    virtual Eigen::VectorXf estimateMeasurement(const State& st) = 0;
    virtual Eigen::MatrixXf Jacobian(const State& st) = 0;
};

class LinearMeasurementModel:public MeasurementModel{
    Eigen::MatrixXf _H;
public:
    LinearMeasurementModel(const Eigen::MatrixXf& H):_H(H){}
    Eigen::VectorXf estimateMeasurement(const State& st);
    Eigen::MatrixXf Jacobian(const State& st){ return _H;}
};
//Not Needed now
class NonlinearMeasurementModel:public MeasurementModel{
    // Eigen::VectorXf MeasurementFunction(const State& st);   
public:
    Eigen::VectorXf estimateMeasurement(const State& st);
    Eigen::MatrixXf Jacobian(const State& st);
};



class Radar{
public:
    static double constexpr _lambda{0.015};

};




#endif