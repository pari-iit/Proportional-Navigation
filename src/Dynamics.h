#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <vector>
#include <map>

class State{
    double _t;//time
    std::vector<double> _x;//state
    std::vector<std::vector<double > > _P;//covariance

public:
    State(const double& t,const std::vector<double>& x, const std::vector<std::vector<double> >& P):_t(t),_x(x),_P(P){}
    ~State(){
        printf("State at %f destroyed.\n",_t);
    }
    State(const State& that):_t(that._t), _x(that._x),_P(that._P){}
    State& operator=(const State& that){
        _t = that._t;
        _x = that._x;
        _P = that._P;
        return *this;
    }

    State(const State&& that):_t(std::exchange(that._t,0)), _x(std::move(that._x)),_P(std::move(that._P)){}
    State& operator=(const State&& that){
        _t = std::exchange(that._t,0.0);
        _x = std::move(that._x);
        _P = std::move(that._P);
        return *this;
    }

    double t() const{ return _t;}
    std::vector<double> x() const {return _x;}
    std::vector<std::vector<double> > P() const {return _P;}

};
class Dynamics{
public:
    virtual void propagate(State& s,const double& dt) = 0;    
};





#endif