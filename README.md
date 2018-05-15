# Path Planning Project
   
### Simulator.
You can download the Simulator which contains the Path Planning Project from the [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) [releases tab].

### Project Goals
In this project goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Pipeline Description

As goal of the project is to generate a trajectory in form of set of waypoints and send it to simulator. simulator uses a perfect simulator to follow more on it is described in details section. We are getting our Main car's localization data , previous path data and sensor fusion data for vehicle around us as described below.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

### Undestanding Constants

To start with the implementation the first few lines of code describes the constants used in thi project

`NUM_OF_LANES 3` : Number of lane on each side of the road which is 3 here.

`LANE_WIDTH 4.0` : Lane of each lane in width is 4m

`MPH_TO_MPS 0.44704` : Main car's speed provided by the simulator is in MPH. constant is used to convert it into Meters per seconds as position of the car and other distance are given in meters

`TIME_BETWEEN_WAYPOINTS 0.02` : Simulator travel a distance between 2 consecutive waypoints in 0.02 seconds

`MAX_VELOCITY_IN_MPH 45` : Maximum allowed Speed is 50 MPH in simulator , but to have some buffer I have set it to 45 MPH

`MAX_ACCELERATION 7 ` : Max allowed acceleration is 10 mps^2 kept it 7 mps^2 to have some buffer.

`FRONT_CLEAR_DISTANCE_FOR_LANE_CHANGE 30` : It defines clear distance (without any vehicle) in front of our car, This distance of 30m is used at the time of making lane change decision.

`REAR_CLEAR_DISTANCE_FOR_LANE_CHANGE 10` : It defines clear distance (without any vehicle) in front of our car, This distance of 30m is used at the time of making lane change decision.

`BUFFER_DISTANCE_TO_FRONT_VEHICLE 30` : It defines the distance to keep from a vehicle in front.

`NUM_WAYPOINTS 50` : Total number of waypoints sent to simulator is 50. which is enough to have smooth trajectory and to cop up with changing  postion of surrounding vehicles.

### Trajectory Generation

Our target here is to be lane and keep driving straight smootly if there is no slow moving vehicle in our front. To be in the same lane we have to maintain some fix distance  from the center of the road. to maintain this distance I am utilizing highway waypoints given in a file. we are also using Frenet Cordinate system to make math easy to understand. 

##### Making Trajectory Smooth
As these highway points are sparse, relying on them directly may introduce sharp change in the orientation of the car between 2 consecutive waypoints, to overcome this problem I am using spline which is a piecewise polynomial It make trajectory smooth and easy to follow without violating jerk criteria.

			tk::spline s;
			s.set_points(anchor_ptsx,anchor_ptsy);
			//---------------------------- Generating Waypoints -----------------------
			for(int i = 0; i < 50-prev_path_size; i++)
			{	  
				    float way_x = (i+1)*dist_inc;
				    float way_y = s(way_x);
					 
					// cout<<"way_x : "<<way_x<<" way_y : "<<way_y<<endl;
					 
					//shifting to global map cordinate system
					float new_x = way_x*cos(ref_yaw)-way_y*sin(ref_yaw)+ref_x;
					float new_y = way_x*sin(ref_yaw)+way_y*cos(ref_yaw)+ref_y;
					
					// cout<<"new_x : "<<new_x<<" new_y : "<<new_y<<endl;
					 
				    next_x_vals.push_back(new_x);
					next_y_vals.push_back(new_y);
	
			}

To also make the trajectory continous and consistent with the previous path , I am utilizing remaining waypoints yet to be covered from the previous path and generating only remaining new waypoints. Total number of waypoints sent to simulator is 50. which is enough to have smooth trajectory and to cop up with changing  postion of surrounding vehicles.

##### Achieving constant Speed

As we driving on structured environment a highway, we have to abide by rules and have to maintain a speed of nearly 50 MPH pn an average , should not cross this speed limit as well.

Here in simulator time taken to travel from one waypoint to next is fixed i.e 0.02 seconds. So to maintain we need to space waypoints such that it result in a constant speed. I am generating waypoints at fixed distance inteval from the current position as shown in below code line

`399 double dist_inc = ref_vel*MPH_TO_MPS*TIME_BETWEEN_WAYPOINTS; //in meters`

##### Lane Change Implementation

If we have a slow moving vehicle in front of us. I am  trying to check a free left and right Lane. If there are no vehicle in 30m front and 10m back , I am changing the lane. If there are any I am following the front car with slower speed till we find a lane to change.(Line number 315)

			if(is_slow_car_in_front){
				int left_lane = lane-1;
				int right_lane = lane+1;
				if(left_lane >= 0 && is_lane_safe_to_change[left_lane]){
					lane = left_lane;
				}else if(right_lane < NUM_OF_LANES && is_lane_safe_to_change[right_lane]){
					lane = right_lane;
				}
				else if(ref_vel > target_velocity){
					ref_vel -= MAX_ACCELERATION*TIME_BETWEEN_WAYPOINTS/MPH_TO_MPS;
				}
			}
			else {
				target_velocity = MAX_VELOCITY_IN_MPH;
				if(ref_vel < target_velocity){
					ref_vel += MAX_ACCELERATION*TIME_BETWEEN_WAYPOINTS/MPH_TO_MPS;
				}
			}


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


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


