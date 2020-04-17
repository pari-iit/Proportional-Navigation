
# Proportional Navigation Repository

This project creates a simulator for **proportional navigation (PN)** guidance for aerial, terrestrial and naval vehicles. Proportional Navigation has been widely used for missile interception. It was widely used during Iraq war for pinpoint target homing. More mathematical details has been omitted and interested readers are directed to the [Wikipedia page](https://en.wikipedia.org/wiki/Proportional_navigation).

In this project I have simulated multiple targets (not necessarily missiles) to be intercepted by multiple interceptors. The targets and interceptors are simulated concurrently. Number of targets are assumed to be the same as number of missiles. Further, the code works for both 2D and 3D. Hence if you have two robots, you can use this for rendezvous problem. All code have been developed from scratch from the boilerplate CPPND starter repo. 

## Algorithm followed for simulation.

1. Number of targets and interceptors ascertained. 
2. Each interceptor assigned to a target. 
3. The following steps are executed in a loop until all the targets are intercepted.
    1. Target moves one step.
    2. Interceptor observes the target using a radar.
    3. Interceptor uses Kalman filter or an extended Kalman filter to estimate target states. 
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
* Requirements for postprocessing simulation results.
  * Python >= 2.7.12
    * All OSes: [click here for download and installation instructions](https://www.python.org/downloads/)
  * NumPy >= 1.16.6 
    * All OSes: [click here for download and installation instructions](https://numpy.org/)
  * Matplotlib >= 2.0.0 
    * All OSes [click here for download and installation instructions](https://matplotlib.org/users/installing.html)

## Basic Build Instructions

1. Clone this repo using `git clone https://github.com/pari-iit/Proportional-Navigation.git`
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./simulator` _`<inputfile>`_.
    * Example: `./simulator ../Data/inputfile_3d.txt`
    * **The _`<inputfile>`_ argument is optional.** If not provided it will use the default file which is `inputfile_3d.txt`. Format of the _`<inputfile>`_ described in `Data/README_inp.md`.
5. Install dependencies for postprocessing: `cd .. && chmod 777 INSTALL.sh && ./INSTALL.sh`
6. Post processing the output:  `cd Data/ && python3 postprocessor.py <dof>`
    * The `<dof>` refers to the degrees of freedom of the system simulated.
    * Example command: `cd Data/ && python3 postprocessor.py 3`
    * The outputs are written in files `Data/res_*.txt`, where `*` refers to the target number.

Right now I am in the process of migrating the output mechanism to ROS and Gazebo, so that simulation outputs could be viewed in real time. However, I do not think I will be able to do that before June, 2020.  

## Class Structure

* `class SimManger` is the main class which manages the simulation. The function `bool runSimulation(const std::string& opdir)` is the function that handles the entire simulation.
* `class Simulator` simulates the target(s) using the inputs provided.
* `class MeasurementModel` takes in the target states and generates radar measurements by adding Gaussian white noise to them. The outputs are of type `Measurement` defined in `class Measurement`. These are the observations made by the radar onboard the interceptor.
* `class KalmanFilter` takes in the measurements generate and provides estimates of target position.
* `class ProNav` in `Controller.h` uses the estimated target position and own position to generate control signal.
* The generated control is then used to move the interceptor towards the target using `class Simulator`.

An important point to note is that the target simulation is done only to generate measurements for the Kalman filter in the interceptor. Target measurements can as well be obtained from other sources such as real radar simulation.

## Project Rubric Points Addressed

* **Loops, Functions, I/O**
  * The project demonstrates an understanding of C++ functions and control structures.
  * The project reads data from a file and process the data, or the program writes data to a file.

* **Object Oriented Programming**
  * The project uses Object Oriented Programming techniques.
  * Classes use appropriate access specifiers for class members.
  * Class constructors utilize member initialization lists.
  * Classes abstract implementation details from their interfaces.
  * Classes encapsulate behavior.
  * Classes follow an appropriate inheritance hierarchy.
  * Overloaded functions allow the same function to operate on different parameters.
  * Derived class functions override virtual base class functions.
  * Templates generalize functions in the project.

* **Memory Management**
  * The project makes use of references in function declarations.
  * The project uses destructors appropriately.
  * The project follows the Rule of 5.
  * The project uses move semantics to move data, instead of copying it, where possible.
  * The project uses smart pointers instead of raw pointers.

* **Concurrency**
  * A promise and future is used in the project.
  * A mutex or lock is used in the project.

## License

The project is distributed under the BSD-3 clause license.