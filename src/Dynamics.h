#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <vector>
#include <map>

class State;
class Dynamics{
public:
    void propagate(State& s,const double& dt) = 0;    
};





#endif