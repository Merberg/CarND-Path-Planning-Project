# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Overview

This repository contains the code needed to complete the Path Planning project for Udacity's Self-Driving Car Nanodegree.  When run, the executable created in this project navigates a simulated vehicle driving on a highway in the [Term 3 Simulator](https://github.com/udacity/self-driving-car-sim/releases).


## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* [Term3 Simulator](https://github.com/udacity/self-driving-car-sim/releases)


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


---

[iStates]: ./data/BehaviorStates.png "Behavior State Machine"


## Project Details

The main loop of the executable steps through the following segments of logic, with information feeding forward into the next step:
1. Vehicle Traffic Tracking
2. Behavior Planning
3. Trajectory Generation

### 1. Vehicle Traffic Tracking

Sensor fustion data is used to determine if a collision is possible; `CarInFront_s - Ego_s < 30.0 m`.
It is also used to check if there is room when the ego vehicle wants to pass:
```
  clearance = abs(ego_s-check_s);

  aheadTooClose = clearance < 30.0 m;
  aheadTooSlow = (clearance < 40.0 m) & (check_vel < ego_vel);
  behindTooClose = clearance < 10.0;
  behindTooFast = (clearance < 20.0) & (check_vel > (TARGET_VEL-5));
  passingRoom = (aheadTooClose || aheadTooSlow || behindTooClose || behindTooFast) ? false : true;
```
Adjusting these clearances can change the ego vehicle from a cautious driving waiting for plenty of room to an inconsiderate traveler that cuts off others.

### 2. Behavior Planning

A state machine consisting of three unique states are utilized for lane change logic:

![iStates]

The 'Check for dropped IDs' action monitors the tracked vehicles to ensure all are accounted for in the event of a sensor fusion error.

### 3. Trajectory Generation

Following the example in the walk-through, the [spline function](http://kluge.in-chemnitz.de/opensource/spline/) enabled the creation of a driveable trajectory.  To avoid sudden changes between the .02 seconds main loop cycles, the last two previous points are reused in the new path.  Additional speed increases are factored into spline point creation when getting back up to the target speed after slow driving (like at the start or post-passing).
