#ifndef CONTROLLER_H
#define CONTROLLER_H


#include <vector>
#include <iomanip>
#include <utility>
#include <unordered_map>
#include "Dynamics.h"



class Control{
    double _t;
    std::vector<double> _u;

    public:
        Control(const double& t, const std::vector<double>& u):_t(t),_u(u){}

        //IMPLEMENT RULE OF FIVE HERE
        ~Control(){
            printf("Control at %f destroyed.\n",_t);
        }
        Control(const Control& that):_t(that._t), _u(that._u)
        {}
        Control& operator=(const Control& that){
            _t = that._t;
            _u = that._u;
            return *this;
        }
        Control(const Control&& that) :
            _t(std::exchange(that._t,0)),
            _u(std::move(that._u)){}
        Control& operator=(const Control&& that) {
            _t = std::exchange(that._t,0);
            _u = std::move(that._u); 
            return *this;
        }

        double time() const {return _t;}
        std::vector<double> control() const {return _u;}
        
};


//Abstract class. Can define any kinds of controllers inside.
class Controller{
public:

    virtual std::vector<Control> generateControl(const std::vector<State>& st) = 0;
};


#endif