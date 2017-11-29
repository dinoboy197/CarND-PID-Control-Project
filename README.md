# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Video of vehicle control

Please see a video of the PID controller I implemented in this project controlling the steering of the vehicle in the Linux Term 2 simulator [here](https://youtu.be/BH02_HSZ-eQ).

## Implementation details

The main implementation pieces of this project include:

* Basic PID controller for steering
* Training module to implement coordinate descent for PID parameter optimization

### PID controller

The [PID controller (class `PID`)](src/PID.cpp) for steering is implemented in a standard fashion, with error adjustments based on the sum of:
* error proportional to a constant (P)
* error delta since previous step proportional to a constant (D)
* total error summed over each step proportional to a constant (I)

### Training module

A [training module (class `TrainPID`)](src/TrainPID.cc) is implemented to optimize the PID constant parameters based on the [coordinate descent algorithm](https://en.wikipedia.org/wiki/Coordinate_descent). The algorithm optimizes each parameter in the error function independently, moving it up or down to find a small improvement in error. This method will find local minima of the error function overall, meaning that the parameters found are not guaranteed to be the best parameters to minimize error, but will be based on starting parameters given.

The coordinate descent algorithm is embedded in a framework to ensure that the total error measurement for parameter setting ends up taking the entire track into account, not just a small portion of it. However, small portions of track are useful for initial parameter and parameter delta setting in the early moments of the algorithm run, to prevent the vehicle from leaving the drivable portion of the track surface. The TrainPID module ensures that if the vehicle appears to be leaving the track surface, the last best parameters are re-installed in the PID controller temporarily, until the vehicle returns to a safe driving state. In that case, those particular parameters that caused the vehicle to lose control are then invalidated as candidates for possible best selection, even if their overall error rate was lower than the previous best error rate.

## Results

Using the training module, and with the inital parameters of p = 0.1, i = 0.01, d = 1.0, the training module arrived at a local optima of error with final parameter values p = 0.127777, i = 0.00937592, d = 1.03501 (in the Linux simulator) and p = 0.3011, i = 0.00110, d = 4.8013 (in the Mac simulator). The two simulators provide different data (different vehicle timestep sizes and angles) to the control program; hence, different values are necessary.

The P (proportional) steering control ensures that steering corrections are proportional to the cross track error of the vehicle to the center of the lane. It is easy to verify this on its own; when the vehicle is travelling away from the center of the lane, the steering commands direct the vehicle back toward the center. However, with this value set and the integral and differential values at zero, the vehicle sways wildely back and forth through the track.

The D (differential) steering control attempts to limit the wild sway of the vehicle as it corrects errors by counter-steering. As steering is changed to correct for error, adjustments are reduced by the amount of change between each time step in the simulation. This attempts to prevent the vehicle from overshooting the center of the lane during a correction.

The I (integral) steering control corrects for steering bias of the vehicle. The vehicle naturally "pulls" to the side when the steering command is 0.0; this ensures that the vehicle drives relatively straight.

***

Overall, these settings for the PID controller ensure that the vehicle drives continuously around the track without hitting any road hazards or leaving the drivable portion of the road surface. One caveat is that in the first few seconds of driving, while the vehicle is under 30 mph, the vehicle does sway from side to side.


--

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

1. Compile: `cmake .. && make`
1. Run it: `./pid`. 
1. Type m or l depending on which simulator you are using.

