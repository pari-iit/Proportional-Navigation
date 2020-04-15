
#CPPND: Proportional Navigation Repository 

This project creates a simulator for **proportional navigation (PN)** guidance for aerial, terrestrial and naval vehicles. Proportional Navigation has been widely used for missile interception. It was widely used during Iraq war for pinpoint target homing. More mathematical details has been omitted and interested readers are directed to the [Wikipedia page](https://en.wikipedia.org/wiki/Proportional_navigation).

In this project I have simulated multiple targets (not necessarily missiles) to be intercepted by multiple interceptors. The targets and interceptors are simulated concurrently. Number of targets are assumed to be the same as number of missiles. All code have been developed from scratch from the boilerplate CPPND starter repo. 

## Algorithm followed for simulation.
1. Number of targets and interceptors ascertained. 
2. Each interceptor assigned to a target. 
The following steps are executed in a loop until all the targets are intercepted.
  1. Target moves one step.
  2. Interceptor observes the target using a radar.
  3. Interceptor uses Kalman filter to estimate target states. 
  4. Interceptor generates PN control signal/guidance law from estimated states. 
  5. Interceptor moves one time step using calculated control signal/guidance law. 


## Dependencies for Running Locally
* cmake >= 3.7
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./simulator.
======

