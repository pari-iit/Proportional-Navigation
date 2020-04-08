#include "Measurement.h"
#include <cassert>
#include <random>


Eigen::VectorXf LinearMeasurementModel::estimateMeasurement(const State& st){
    assert (st.x().size() == _H.cols());
    return _H*st.x();
}

/* This is relative measurements. State is the relative state between robots.
relative measurement 
IMPORTANT: 
state is defined as 
s = [(x-x_own), (y-y_own),  (z-z_own),  (vx-vx_own),  (vy-vy_own),  (vz-vz_own)]

Measurements y = [d, fd]
d = \sqrt((x-x_own)^2+(y-y_own)^2+(z-z_own)^2)
fd = -2/\lambda* ((x-x_own)(vx-vx_own)+(y-y_own)(vy-vy_own)+(z-z_own)(vz-vz_own))/d; 

*/
Eigen::VectorXf NonlinearMeasurementModel::estimateMeasurement(const State& st){
    Eigen::VectorXf s = st.x();
// #if _N_STATES == 3
//     Eigen::Vector3f x,v; 
// #elif _N_STATES == 2
//     Eigen::Vector2f x,v;
// #endif
    Eigen::RowVectorXf x(_N_STATES),v(_N_STATES);
    x = s.head(_N_STATES), v = s.tail(_N_STATES);
    //Distance measure
    double d = x.norm();
    //Doppler frequency
    double fd = -2/Radar::_lambda*(x.dot(v))/d;
    Eigen::VectorXf m(_NM);m << d,fd;
    return m;
}
//Same thing now we will add noise to the measurements;
Measurement NonlinearMeasurementModel::generateNoisyMeasurement(const State&& st,const Eigen::MatrixXf& R){    
    std::random_device rd;
    std::mt19937 eng(rd());
    
    Eigen::VectorXf v = estimateMeasurement(st);    
    assert (v.size() == _NM );

    for (int i=0;i < v.size(); ++i){
        std::normal_distribution<double> dist(0, 0.01*R(i,i));
        v(i) = v(i) + dist(eng);
    }

    return {st.t(),v,R};
    
}

/* Jacobian defined as 
// H = [(x-x_own)/d      (y-y_own)/d    (x-x_own)/d    0       0        0]
H(1,0) = -2/lambda[(vx-vx_own)/d-(vx-vx_own)*(x-x_own)^2/d^3]
H(1,1) = -2/lambda[(vy-vy_own)/d-(vy-vy_own)*(y-y_own)^2/d^3]
H(1,2) = -2/lambda[(vz-vz_own)/d-(vz-vz_own)*(z-z_own)^2/d^3]

H(1,3) = -2/lambda[(x-x_own)/d]
H(1,4) = -2/lambda[(y-y_own)/d]
H(1,5) = -2/lambda[(z-z_own)/d]

*/
Eigen::MatrixXf NonlinearMeasurementModel::Jacobian(const State& st){
    Eigen::VectorXf s = st.x();
// #if _N_STATES == 3
//     Eigen::RowVector3f x,v;
// #elif _N_STATES == 2
//     Eigen::RowVector2f x,v;
// #endif
    Eigen::RowVectorXf x(_N_STATES),v(_N_STATES);
    x = s.head(_N_STATES), v = s.tail(_N_STATES);
    Eigen::MatrixXf J(2,s.size());
    //Distance measure
    double d = x.norm();
    //Doppler frequency
    double fd = -2/Radar::_lambda*(x.dot(v))/d;
    J.row(0) << x/d, Eigen::RowVectorXf::Zero(_N_STATES);
    double K = -2/Radar::_lambda;
    for(int i=0;i<_N_STATES;++i){
        J(1,i) =  K*(  (v(i)/d) -  (v(i)*pow(x(i),2)/pow(d,3))  );
    }

    for(int i=0;i<_N_STATES;++i){
        J(1,i+3) =  K* (x(i)/d) ;
    }


    return J;
}


