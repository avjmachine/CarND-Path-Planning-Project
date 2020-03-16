# CarND-Path-Planning-Project Writeup
Self-Driving Car Engineer Nanodegree Program

---  

[//]: # (Image References)

[image1]: ./writeup_images/spline_description.png "Description of Spline used for trajectory generation"
[image2]: ./writeup_images/flowchart.png "Flowchart for approach used here for path planning"

## Introduction

The goal of this project is to safely navigate around a virtual highway with other traffic, using a C++ program for path planning. There is a 50 MPH speed limit which must be followed. The car's localization and sensor fusion data will be provided as inputs from the simulator to the C++ program. It is also provided a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, by passing slower traffic when possible. The car should avoid hitting other cars at all cost. It should also drive inside of the marked road lanes at all times, except during the brief moments while switching lanes. 

The car should be able to make one complete loop around the 6946m highway and stick close to the 50MPH speed, and must slow down only when there obstructing traffic and lane change is not feasible. Also, the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


## Approach used for Path Planning

The complex path planning required for a long drive such as this one of about 4.32 miles is made easier by breaking it into smaller parts. The whole journey is broken into multiple shorter paths, each of which can be traversed in 50 blocks of 0.02s (which sums up to ~1s). The path for approximately next 1s in future is planned in advance at once at each step and fed into the simulator. This path data is given in the form of x and y map coordinates to be traversed by the vehicle. The vehicle controller makes sure that the vehicle drives through each of these points. Though 50 points are given, not all may be used by the simulator in the same step. The points which are unutilized are carried over to the next step and only the remaining points are calculated. For example, if there are 45 unutilized points from the previous step, only the remaining 5 points are required to be calculated. At each step, the simulator is given just 50 points of trajectory data that can be used for approx. next 1s in future.

The approach used in each step can be divided into 2 main parts. The first part is deciding the manoeuvre and the second is generating the trajectory.

The first part of deciding the manoeuvre is done using inputs such as the localization data and sensor fusion data. The localization data gives the position of the ego vehicle. The sensor fusion data gives the positions and velocities of the other vehicles on the road in the vicinity. Combining and comparing both these data helps in finding out if it is safer, efficient and feasible to remain in the same lane or change lanes. It also helps in deciding if braking is required to prevent collisions or if acceleration is required to make the vehicle drive faster when the lane is free. The lane that is safe as well as fast is searched for and selected. The following rules, not necessarily in the same order, are used for lane changes:

1. If there is a vehicle detected in front within 25m, consider changing lanes. Also, start braking to reduce risk of collision.
2. Change lanes only below 45mph speed to avoid exceeding max acceleration limits. Lane change at higher speeds than this are observed to cause accelerations that exceed the limits, with the current approach used here.
3. Change lanes only above 25mph speed to avoid very slow lane changes. Such slow lane changes are observed to increase the risk of being outside lane for longer periods or increase risk of collisions from rear.
4. Change lanes only when the previous lane change manoeuvre is complete. This avoids continuous changing of lane, leading to confusions, too frequent lane shifts and a state of limbo.
5. Check the neighbouring lanes for safety and speed before confirming lane change. If an immediate neigbouring lane is safe, but slow due to obstructing traffic, check if the lane next to it is safe and fast, and enter into this third lane using the immediate neighbouring lane as a temporary transitioning lane.
6. If in centre lane, check for the left lane first and if not clear, check for the right lane next.
6. Check for safety during lane shifting by looking both ahead and behind in the destination lane. 10m is the absolute distance ahead and behind that needs to be clear, regardless of the vehicle speeds. 30m is the distance required to be clear if there is a faster vehicle from behind or slower vehicle ahead.
7. Don't accelerate or brake during lane change. This is to reduce the higher resultant acceleration due to combined cornering and acceleration.

The second part in this planning process is generating a smooth trajectory till the next destination. This is done by using a spline. The spline is generated using 5 anchor points. The first 2 anchor points are taken from the last 2 points from the previous step's path or in case of no previous points, they are estimated by reverse calculating from the current position. The remaining 3 anchor points are kept at 25m, 50m and 75m (spaced significantly far off on the map to construct a smooth spline). 25m is selected since that is the distance value checked for avoiding collisions and hence can be considered as a safe point to move to (there is some buffer of safety built into this, since even if the vehicle detected ahead comes to an abrupt standstill though impossible, we are still safe!). The anchor points are specified in the Frenet coordinates since Frenet coordinates are easier to specify waypoints on the map from a path planning and trajectory generation perspective. These are later converted into the vehicle local coordinates for avoiding numerical issues with splines (splines are easier to work with when there are no multiple y values for each x value!). Once converted to vehicle coordinates, the spline is fitted.

The first anchor point is the target or destination for the current step of path planning. Using this first anchor point as the target end point, the fitted spline is then divided into smaller multiple pieces. The length of these smaller pieces is decided by the distance to be covered at the desired speed in every 0.02s time interval. Every subsequent point marked/selected on this spline is traversed after an interval of 0.02s. This spacing and division of the spline controls the speed, acceleration and jerk.

The length of each small piece of the spline is given by the formula `l = desired_speed*0.02s`. The number of splits is given by the formula `N = length till the target end point/l`. The x axis of this spline is divided by this N no. of splits and its y value is calculated at every split using the spline function. Each marked point is converted converted back to the map coordinates and fed to the simulator till a total of 50 points including the previous path points is reached at every step. This trajectory generation using splines is illustrated in the following figure:

![alt text][image1]

This entire approach can be compactly represented in the form of a flowchart (without the details) as shown below:

![alt text][image2]


## Running the Code

The following are the requirements for running the code:

### Simulator

The Term3 Simulator for the Path Planning Project can be downloaded from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```   

### Basic Build Instructions

To build and run the code, follow the steps given below:

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


## Implementation - Description of the Code


### Input Data

The following are the input data provided for path planning.

#### Map of the highway

The map data is provided as a text file in `data/highway_map.txt`. Each waypoint in the list contains the following values:

x  - waypoint's x position in map coordinates

y  - waypoint's y position in map coordinates

s  - distance along the road to get to that waypoint in meters

dx - x component of unit normal vector pointing outward of the highway loop

dy - y component of unit normal vector pointing outward of the highway loop


#### Main car's Localization Data 

The following values are provided as localization data for the ego vehicle by the Simulator to the C++ program:

["x"]    -  x position in map coordinates

["y"]    -  y position in map coordinates

["s"]    -  s position in Frenet coordinates

["d"]    -  d position in Frenet coordinates

["yaw"]  -  yaw angle in the map

["speed"]-  speed in MPH


#### Previous path data given to the Planner

The Simulator also provides the following path data from the previous step to the C++ program, but with the processed points removed. This list of points not processed in the previous step are processed in the subsequent step and helps in smoothing transitions to future path trajectories. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

["end_path_s"] The previous list's last point's Frenet s value

["end_path_d"] The previous list's last point's Frenet d value


#### Sensor Fusion Data

Sensor fusion data is provided by the Simulator to the C++ program.

["sensor_fusion"] is a 2d vector of other cars detected by the sensor fusion module on the highway and contains the following 7 values for each car:

0. unique ID of the car 
1. car's x position in map coordinates
2. car's y position in map coordinates
3. car's x velocity in m/s
4. car's y velocity in m/s
5. car's s position in Frenet coordinates
6. car's d position in Frenet coordinates. 


### Output Data

The following data is provided by the C++ program to the Simulator after planning the path and generating a trajectory for the next step:

["next_x"]  -  a list of x coordinates of the next set of points forming the trajectory

["next_y"]  -  a list of y coordinates of the next set of points forming the trajectory


### Code Description

The code can be found in the `src` directory. The `main.cpp` and `helpers.h` files with skeleton code are modified for implementing this project. A brief explanation of each of the files in this `src` directory is given below:

1. `main.cpp` - This file contains code to interface with the Simulator, read in input data, process the input data for path planning and to generate trajectories. As a first step, it reads in map data (lines 20-52) from `data/highway_map.csv` and stores the waypoint data in a vector. It communicates with the Simulator using SocketIO and JSON messages (lines 59-76 and lines 358-389). In every communication cycle with the Simulator, it reads in Localization data and Sensor Fusion data (lines 78-95). It then considers unutilized points from the previous path to be used in the current cycle and also to smooth transitions to the new path points yet to be created (lines 119-164). After this, it checks for conditions to change lanes such as completion of existing lane change, presence of vehicles ahead in the same lane posing a risk of collision, speed requirements for lane change, safety clearance and speed of traffic in destination lane, etc. (lines 171-282). Then based on whether there is a need to apply brakes or scope to accelerate, the speed of the ego vehicle is reduced or increased (lines 284-297). After these checks and controls, there is also code for trajectory generation. This is done by creating a spline based on the destination or target position for each cycle. The spline is used to generate a trajectory (lines 299-353). The spline fitting and trajectory generation is done in the vehicle coordinate system for ease and to avoid numerical issues and is transformed back into the map coordinate system. Finally, the generated set of x,y points forming the trajectory are passed into a json message object (lines 355-356) to be fed into the simulator at the end of each processing cycle.

2. `helpers.h` - This file contains helper functions. One such helper function is `hasData()` to check if a SocketIO event has JSON data. This function is useful while communicating with the Simulator. There are also tiny helper functions to return pi value such as `pi()`, to convert between degrees and radians such as `deg2rad()` and `rad2deg()`. Functions to get the closest waypoint such as `ClosestWaypoint()` or to get the next waypoint, which is the closest waypoint ahead of the vehicle, such as `NextWaypoint()` are also found in this file. Functions like `getFrenet()` and `getXY()` to convert to and from Frenet and map xy coordinates can also be found here. In addition to the above mentioned functions which are already provided, there are also a couple of new functions defined here such as `frontal_collision_risk()`(lines 157-177) and `lane_status()`(lines 179-232). The former checks if there is another car in front of the ego vehicle in the same lane posing a risk of collision, while the latter checks if a given destination lane is (a) unsafe, (b) safe but slow or (c) both safe and fast.
 
3. `spline.h` - This file is downloaded from http://kluge.in-chemnitz.de/opensource/spline/ and contains the spline function in a single header file. It is used for fitting a spline to some anchor points and reading off y values for given x values on the spline. This is used for generating trajectories.

4. `json.hpp` - This file contains code that is used for communicating with the Simulator using json messages.


## Rubric Items

### Compilation
- The code compiles correctly without errors, using cmake and make.

### Valid Trajectories
- The car is able to drive 4.32 miles without incident.
- The car does not cross the speed limit, and drives close to the limit, as long as it is not obstructed by traffic.
- Max. acceleration of 10 m/s^2 and max. jerk of 10 m/s^3 are not exceeded.
- The car does not have collisions.
- The car always stays within valid lanes, except for the time during lane changes, which don't exceed 3 seconds.
- The car is able to change lanes smoothly when required, if it is safe and feasible.

### Reflection
- The writeup section "Approach used for Path Planning" describes in detail how paths are generated.
