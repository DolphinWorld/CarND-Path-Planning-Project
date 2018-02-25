# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
---
![[Record Video][Record.mp4]](screenshot.png)

*Download [Record.mp4](Record.mp4?raw=true) for the full video.*

---

## Implementation ##

### Read in highway map data and use it as map baseline ###
Highway waypoints data is from `highway_map.csv`, which will be serving as baseline to transform from Frenet to Cartesian.

### Handle `Telemetry` event every 0.2 second. ###

Every 0.2 second, `telemetry` event will be triggered. When data of the event is a json string, which contains current car context, previous path, sensor information. The car context information includes `x`, `y`, `s`, `d`, `yaw` and `speed` of current car. The sensor information contains position and speed of all the surranding objects in Frenet coordination. 

#### Find cars in nearby and current lanes ####
We care about other cars in three lanes, left, right and current lane. For current lane, we need to find out whether the car in front of us are too slow and may collide to my current car; for left and right lanes, we need to detect whether it is safe to switch lane to.

This line is to find all the cars in neighbor and current lanes:
```c
if (d < (2 + 4*(lane + 1) + 2) && d >(2 + 4 * (lane - 1)- 2))
```

And this is to detect whether it is left or right or current lane. `diff == 1` is for right lane, `diff == -1` is for left lane and `diff == 0` is for current lane.

```c
int diff = 0;
if (d >= (2 + 4*lane + 2)) {
    diff = 1;
}
if (d < (2 + 4 * lane - 2)) {
    diff = -1;
}
```

#### Detect collision ####

If the nearby car is in the same lane, in front of current car, and the distance shorter than 30 meter, we will mark the `too_close` flag to true for further consideration.

```c
if (diff == 0 && (check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
    too_close = true;
}
```

#### Detect the safety of neighbor lanes ####

If the `s` distance between the car in the neighbor lane is between -5 meter to 20 meter, then it is treated as not safe. It will then set the safe_left or safe_right to false to avoid changing to that neighbor lane.

```c
double distance = check_car_s - car_s;
bool safe = distance > 20 || distance < -10;
if (!safe) {
    if (diff == -1) safe_left = false;
    if (diff == 1) safe_right = false;
}
```

#### Plan next action ####

If `too_close` flag is `true`, we will first try to switch to neighbor lane if it is safe. If the neighbor lanes are not safe to switch to, then we will slow down the car.

If `too_close` is not `true`, and the car speed is slower than the limit, we will speed up slowly. 

Here is the code:

```c
if (too_close) {
    if (lane > 0 && safe_left) {
	lane--;
    } else if (lane < 2 && safe_right) {
        lane++;
    } else if (too_close) {
	ref_val -= .19;
    }
}
if (!too_close && ref_val < 49.5) {
    ref_val += .19;
}
```


### Generate Trajectory ###
We will use at least five points to feed to spline to calculate the trajectory. Sensor returns previous paths, if there is enough data points, we will use the last two; otherwise we will calculate using the current car position.

```c
if (prev_size < 2)
{
       double prev_car_x = car_x - cos(car_yaw);
       double prev_car_y = car_y - sin(car_yaw);
       ptsx.push_back(prev_car_x);
       ptsx.push_back(car_x);

       ptsy.push_back(prev_car_y);
       ptsy.push_back(car_y);
} else {
       ref_x = previous_path_x[prev_size - 1];
       ref_y = previous_path_y[prev_size - 1];
       double ref_x_prev = previous_path_x[prev_size - 2];
       double ref_y_prev = previous_path_y[prev_size - 2];

       ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

       ptsx.push_back(ref_x_prev);
       ptsx.push_back(ref_x);

       ptsy.push_back(ref_y_prev);
       ptsy.push_back(ref_y);
}
```

The other three points are predicting from current car position and the lane we are driving to. The lane could be current lane, or left (previous lane - 1) lane, or right (previous lane + 1) lane.

```c
vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);


ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
```

Then we will convert the locaton information from absolute angle to car angle, and send to spline to using set_points to calcluate.
```c
 for (int i = 0; i < ptsx.size(); i++) {
         double shift_x = ptsx[i] - ref_x;
         double shift_y = ptsy[i] - ref_y;
         ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
         ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

 }
 tk::spline s;

 s.set_points(ptsx, ptsy);

```

Eventually we pass the information to the variable `next_x_vals` and `next_y_vals` and set to `msgJson` to control the car driving trajectory.

---

## Project Instructions 

---

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

