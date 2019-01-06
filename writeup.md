This documentation is prepared for introducing the model and implementation for the **Udacity Path-Planning Project**.

## Task Description

The goal of this project is to build a path planner creating smooth, safe trajectories for the car to follow. 
The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.
Also, safety should be the key factor of the path planner to avoid collision.

## What is the Path to be Planned

A trajectory is constituted by as set of (x, y) points. Every 20 ms the car moves to the next point on the list.
Since the car always moves to the every next way point after 20ms, the **velocity** of the vehicle is calculated by how much future waypoints are spaced.
In other words, the larger the space between way points, the faster speed the car will reach.

## Rubric

Rubric points along with an explanation of how constraints are satisfied are listed as following.

- **The car drives according to the speed limit**

    A [reference velocity variable](src/main.cpp#L192) 0 is used to keep track of current speed of the vehicle. 
    Also, [another constant variable](src/utils.h#L14) 49mph is used to make sure the car never exceed the maximum speed of 50 mph.

- **Max acceleration and jerk are not exceeded (a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3).**
    This is obtained by setting the initial speed of vehicle to zero, and gradually accelerating (4.2m/s^2) until the vehicle has reached the maximum speed allowed.
    When the vehicle gets too close to the forward vehicles , it slows down gracefully with the same negative acceleration. 
    Moreover, when the vehicle's speed is below 25mph, the acceleration is set as 0.4mph/20ms (about 8.4 m/s^2).
    This part is implemented in [main.cpp](src/main.cpp#L308-L316).

- **Car does not have collisions.**
    Under the assumption that other cars drive reasonably well, there are several risks of collisions:
    * hit another car at *rear-ending*.
    * hit another car when switching lane.
    * get hit when switching lane.  
    
    The first issue is addressed using sensor fusion data to make sure that no other car is dangerously close in the same lane. [main.cpp](src/main.cpp#L254-L258) 
    If this is the case, the car [gracefully decelerates](src/main.cpp#L308-L309) until the situation gets safer.
    Another 2 possibly dangerous situation will be examined later in the one of *change lanes*. 

- **The car stays in its lane, except for the time between changing lanes.**

    One of the first things that we would demand from a self-driving car could drive inside a certain lane. 
    
    For the task of lane keeping, we could rely on *Frenet coordinates*, which measure longitudinal (s) and lateral (d) displacement along the road and are much easier to deal with than euclidean ones. 
    
    The transformation between frenet and euclidean coordinates are provided in [main.cpp](src/main.cpp#L85-L154).
    In order for the car to keep the lane, three segments containing future waypoints are generated equally spaced 30m, 60m and 90m before the car. 
    Notice that these waypoints are created using the specific `lane` variable as well as the lane width to make sure that the lateral displacement is just in the middle of the lane the car is driving on. [main.cpp](src/main.cpp#L355-L361)   
    
    These are quite far apart one from the other, so in order to obtain a smooth trajectory the spline is used to interpolate intermediate locations, as introduced in the class.
    
- **The car is able to change lanes**

    When it makes sense to do, the vehicle will try to change lane. This will happen in case a slower moving car obstructs traffic lane. 
    A simple finite state machine dealing with safe lane changing is implemented at [these lines](src/main.cpp#L246).
    As sensor fusion data has collected information about nearby vehicles for us, when there're vehicles travelling on the same lane, and getting dangerously close to our car, we'll prepare for a lane changing.
    When the car has entered the state of `prepare_for_lane_change`, in order to safely change lane, the car has to check if the left or right lane is sufficiently clear (this means both the front and the behind direction of the target lane should satisfy the safe margin for our vehicle to change to) of traffic for a safe lane change. 
    If at least one lane free is found, the state flag to signal that a safe lane changing is now possible.
    If there're 2 free lanes at the same time, we'll drop the busy one, as we have the information of vehicle numbers from the sensor fusion data.
    The actual lane change state happens by setting the `lane` variable to the new value. [main.cpp](src/main.cpp#L296-L305). 
    Also, once the lane change is executed, for the next iteration, all the state flags (`prepare_for_lane_change`, `ready_for_lane_change`) would be reset until a new state is trigged.
    
- **Discussion**
    The general implementation takes reference from [repo](https://github.com/ndrplz/self-driving-car/blob/master/project_11_path_planning/src/main.cpp) with a few improvements like different acceleration for low speed and high speed, choosing a less busier lane of 2 available options.
    In one observation, best distance without incidents reaches 10 miles, with lane changes performed, lane keeping and without collision.
