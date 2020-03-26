#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <vector>
#include <stdio.h>

class Measurement{
    double _t;
    std::vector<double> _y;
    std::vector<std::vector<double> > _R;

public:
    Measurement(const double& t, const std::vector<double>& y, const std::vector<std::vector<double> >& R):_t(t), _y(y), _R(R){}

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


};









#endif