#include "Measurement.h"
#include <cassert>
Eigen::VectorXf LinearMeasurementModel::estimateMeasurement(const State& st){
    assert (st.x().size() == _H.cols());
    return _H*st.x();
}

//This is relative measurements. State is the relative state between robots.
Eigen::VectorXf NonlinearMeasurementModel::MeasurementFunction(const State& st){
    Eigen::VectorXf s = st.x();
    Eigen::Vector3f x = s.head(3), v = s.tail(3);
    //Distance measure;
    double d = sqrt(x.dot(x));
    double fd = -2/Radar::_lambda*(x.dot(v))/d;
    Eigen::Vector2f m;m << d,fd;
    return m;
}