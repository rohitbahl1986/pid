# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Reflection


PID control uses a combination of Proportional (P), Integral (I) and Differential (D) controllers.

The **P** controller influences the overall gain in proportion to the cross track error. However, since rate of change of the error is not tracked, a **P** controller leads to oscillating behavior around the reference position.

The **D** controller applies a gain in proportion to the differential of the error i.e. it takes into account the rate of change of the error. Therefore, as the robot approaches the reference trajectory, less gain is applied leading to a stable trajectory near the reference trajectory.

The **I** controller takes into account any systematic bias in the system. It adds up the error over time and applies a gain to compensate for any deviations from the reference trajectory. The trajectory will be stable even without the **I** controller (i.e. no oscillations), however, it will not be the right trajectory since will be off by a bias term.

In this project I have used the **Twiddle** algorithm to continuously find the correct set of weights for **P**, **I** and **D** controller. It is observed that since there is no bias in the system, the weight of the **I** controller quickly drops to **0**. If we set the weight of the **D** controller also to **0**, the car starts to oscillate across the road and eventually skids of the road. The **D** gain stabilizes around **10** while the **P** gain stabilizes around **0.3**. The number of iterations after which the **Twiddle** algorithm is invoked is a parameter which is tuned based on multiple runs. If the parameters are changed too often (around 10 iterations), the car starts to wobble. If the iterations are too large (around 1000), car goes somewhat out of course while taking turns. Finally, a value of **100** has been selected after multiple runs and gives reasonable performance. 
