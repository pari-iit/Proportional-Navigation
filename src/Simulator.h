#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <vector>
#include <thread>
#include <memory>
#include <mutex>
#include "Dynamics.h"

class Dynamics;
class Simulator{
    double dt;
    std::shared_ptr<Dynamics> dynamics;
    std::mutex _mut;
  public:
    State Simulate(const State& st);
};


#endif
