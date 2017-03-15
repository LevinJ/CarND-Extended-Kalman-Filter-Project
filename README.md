# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

---

### Overview
The goals / steps of this project are the following:  

* Complete the Extended Kalman Filter algorithm in C++.
* Apply it to pedestrian position/speed estimation problem 
* Evaluate it again two sampled data, the metrics used is rmse

### Final Result

1. sample data I

![Pedestrian Tracking I](https://github.com/LevinJ/CarND-Advanced-Lane-Lines/blob/master/camera_calibration.png)


Accuracy - RMSE:
0.0269264
 0.024363
 0.335339
 0.370815

2. sample data II
![Pedestrian Tracking II](https://github.com/LevinJ/CarND-Advanced-Lane-Lines/blob/master/camera_calibration.png)


Accuracy - RMSE:
0.170502
0.167365
0.400714
0.630331


Note that the elements in RMSE vector corresponds to [px, py, vx, vy]



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`



