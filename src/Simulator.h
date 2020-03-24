#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <vector>
#include <thread>
#include <memory>
#include "Dynamics.h"

class Dynamics;
class Simulator{
    double dt;
    std::shared_ptr<Dynamics> dynamics;
    public:

};


#endif
