#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <vector>
#include <map>
#include <eigen3/Eigen/Dense>
#include "Controller.h"
#define _N_STATES 2
#define NS (2*_N_STATES)
#define _N_CONTROL 2

class Control;
class State{
    double _t;//time
    Eigen::VectorXf _x;//state
    Eigen::MatrixXf _P;//covariance

public:
    State():_t(0.0),_x(Eigen::VectorXf::Zero(_N_STATES) ),_P(Eigen::MatrixXf::Identity(_N_STATES,_N_STATES) ){}
    State(const double& t,const Eigen::VectorXf& x, const Eigen::MatrixXf& P):_t(t),_x(x),_P(P){}
    ~State(){
        // printf("State at %f destroyed.\n",_t);
    }
    State(const State& that):_t(that._t), _x(that._x),_P(that._P){}
    State& operator=(const State& that){
        _t = that._t;
        _x = that._x;
        _P = that._P;
        return *this;
    }

    State(State&& that):_t(std::exchange(that._t,0)), _x(std::move(that._x)),_P(std::move(that._P)){}
    
    State& operator=(State&& that){
        _t = std::exchange(that._t,0.0);
        _x = std::move(that._x);
        _P = std::move(that._P);
        return *this;
    }

    double t() const{ return _t;}
    Eigen::VectorXf x() const {return _x;}
    Eigen::MatrixXf P() const {return _P;}

    
    void setT(const double& t){ _t = t;}
    void setX(const Eigen::VectorXf& x){_x = x;}
    void setP(const Eigen::MatrixXf& P){_P = P;}


    double getDistance(const State& that){        
        return (_x.head(_N_STATES)-that.x().head(_N_STATES) ).norm();
    }

};
class Dynamics{
public:
//Can be used for continuous and discrete time. In an already discretized system dt = 1
    virtual void propagate(State& s,const Control& u,const double& dt) = 0;    
    virtual void propagate(State& s, const double& dt) = 0;
    virtual Eigen::MatrixXf Q() = 0;
    virtual void Q(const Eigen::MatrixXf& Q) = 0;
};

//Any generalized linear dynamics. Can be used for Kalman filter.
class LinearDynamics:public Dynamics{
    Eigen::MatrixXf const  _F;
    Eigen::MatrixXf _Q;
    Eigen::MatrixXf const _B;    
public:
    LinearDynamics(const Eigen::MatrixXf& F, const Eigen::MatrixXf& Q):_F(F),_Q(Q),_B(Eigen::MatrixXf::Zero(NS,_N_CONTROL)){}
    LinearDynamics(const Eigen::MatrixXf& F, const Eigen::MatrixXf& Q, const Eigen::MatrixXf& B):_F(F),_Q(Q), _B(B){}    
    void propagate(State& s, const double& dt);
    void propagate(State& s, const Control& u, const double& dt);    
    Eigen::MatrixXf Q(){return _Q;};
    void Q(const Eigen::MatrixXf& Q){_Q = Q;};
};


//TODO: This depends on the nonlinear dynamical function used.Current project does not use this.
class NonlinearDynamics:public Dynamics{    
    Eigen::MatrixXf Jacobian(const State& s);
    Eigen::MatrixXf _Q;
public:
    NonlinearDynamics(const Eigen::MatrixXf& Q):_Q(Q){}
    void propagate(State& s, const Control& u, const double& dt);
    void propagate(State& s, const double& dt);
    Eigen::MatrixXf Q(){return _Q;};
    void Q(const Eigen::MatrixXf& Q){_Q = Q;};
};





#endif