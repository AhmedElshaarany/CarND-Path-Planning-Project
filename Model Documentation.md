# Model Documentation
Self-Driving Car Engineer Nanodegree Program
   
In this documentation, the code model for generating paths is described. We will go through the main.cpp file and explain what each code snippet contributes to the path planner.

### Initialization
In lines (13-26) in main.cpp, a few values are defined that are used to tune the performance of the path planner such as lane shift safety distance, speed limit, ...etc. The ego-vehicle is initialized with 0 speed and it should start in the middle lane as shown in lines (216-220).

### Emergency Break Logic
Some times cars change lanes suddenly, and in order to avoid crashes, in lines (276-308) of main.cpp, an emergency brake logic is implemented. The main idea is that when a car suddenly changes lanes and is now in front of the ego-vehicle with a predefined emergency buffer distance, the ego-vehicle reduces its speed at a high deceleration rate without violating the jerk constraint.

### Car Ahead Detection
In lines (311-334), the sensor fusion list is iterated and we check if there are any cars in front of us and within the ego-vehicle's pre-defined buffer distance value. If a car is found, we raise a flag and capture that car's id and process it later on. 

### Car Ahead Processing
Once a car in our lane is detected, the ego-vehicle reduces its speed and maintains the same speed as the car ahead of it until a lane change ins possible as shown by lines (337-350).

### Lane Change Logic
For lane changing, the finite state machine (FSM) approach is used. This means that the logic to do a lane change depends on the ego's current lane.
If the car is in the center lane, we consider both left are right lanes. We use Frenet coordinates to determine which lane has fewer cars ahead of the ego vehicle. Once we calculate those numbers, we get ready to change lanes to the lane with fewer number of cars ahead. Getting ready to do a lane change means checking the lane with the fewer number of cars and checking to see if there is a safe enough gap to do the lane change. A safe gap means that there is enough distance in front of and behin the ego vehicle in the selected lane to do the lane change. We check for the safe gap again using Frenet coordinates. This is shown in lines (353-412) in main.cpp.
If the ego is in the left lane and there is a car in front of it, we check if there is gap wide enough in the center lane to do a safe lane change. This is shown in lines (413-449).
If the ego is in the right lane, a logic similar to left lane is implemented in lines (450-488).

### Path Planner
For path planning, we create a vector of waypoints to create a spline that would be used later on to create the ego's path. As Aaron Brown suggested, it turned out to be a great idea to use previous path points to guarantee the continuity of the ego's path as well as its smoothness. In lines (505-565) we use previous path points as well adding more points to the waypoints vector to create the spline.
In lines(567-610), we create points for the ego's path using the spline we created and add them to the vector of x and y values that are passed to the simulator.
