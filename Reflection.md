#**Path Planning** 

**Path Planning Project**

The goals / steps of this project are the following:
* Write a "behavior module" for a simulated self-driving car to determing the best path it should take on highway driving
* Get the car to circle the loop (over 4 miles!) at least once.
* And report on my understanding of path planning and the process followed 
** This here markdown readme is the written report!


[//]: # (Image References)
[image1]: ./cheat_win.png "A fun way to cheat with using just the straight path trajectory"
[image2]: ./actual_win.png "Screenshot of an actual run that ran for a very long time"

#Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1020/view) individually and describe how I addressed each point in my implementation.  

---

##1. Your code should compile.

I am fairly certain my code compiles and runs without exceptions.

##2. Valid Trajectories

### The car is able to drive at least 4.32 miles without incident.
As per the below screenshot, I can reasonably conclude that my car's behavior is rewarding enough to get around the
track without incident.  Although it's not optimal (I am currently working on attempting to implement the much-harder
path planner as discussed in the lectures), it gets the job done and changes lanes when it is reasonably safe.

The only time it really has a problem is when it gets stuck in a slow lane with a lot of traffic passing it.
The car is a bit conservative about merging in, though if required, I can adjust the parameters of the simple approach
I used for lane changing to make it go a bit faster.  To be fair, this actually mimics my driving style as well,
where I prefer to just let on-coming traffic in the next lane pass me when I want to pass a slow-poke.  I dislike 
in general having to accelerate between two cars and would consider that a riskier move to take than just staying put.  
 
 ![alt text][image2] 

### The car drives according to the speed limit.
No speeding violations have been observed as the car follows the max 49.5, while slowing down for any on-coming traffic. 

### Max Acceleration and Jerk are not Exceeded.
With my current implementation, I observed once where jerk/acceleration was violated when the car attempted to 
move across two lanes at once.  This could be addressed by keeping the car in a target lane before it can decide
to change lanes again.  The finite-state machine implementation would be useful for that. 

### Car does not have collisions.
I did not witness my car crash during the implementation as provided after driving over 30 miles, though I didn't
have my eyes on it the whole time.

### The car stays in its lane, except for the time between changing lanes. ###
No violations are recorded for the car changing lanes, even when changing across two, it does it fairly quickly.

### The car is able to change lanes ###
As mentioned earlier, the car is a little conservative about when it considers the left and right lanes to be clear,
but it definitely changes lanes when approaching a single slow-poke with no other traffic around.

##3. Reflection

### There is a reflection on how to generate paths. ###

The model I implemented primarily follows what was covered in the Q&A video.  It has two main parts that, conveniently,
may be found in two separate methods in the main.cpp. They are ''determine_lane_and_velocity'' 
and ''generate_trajectory_for_lane'', more details in the next two sections.

#### determine_lane_and_velocity #### 
This is a very basic implementation I wanted to try after having gone through the Q&A and had a running vehicle.  
The algorithm could be improved upon, but gets the job done and I would rather spend my energy on implementing the FSM
than fine-tuning this basic one.  All it does is extend the logic for checking if the lane-ahead is clear to 
also check if the left and right lanes are clear, while also ensuring we stay within the boundaries.  If we are 
getting too close to the car in front, then we simply prefer making left changes if the lane is clear, or right-lane changes
and otherwise slowing down.  This is also where the velocity is controlled whereby we increment and decrement by a small amount
to ensure we stay within the necessary jerk parameters later on when we generate the trajectory.

#### generate_trajectory_for_lane ####
This uses a spline library to generate a smooth trajectory that stays within the acceleration and jerk parameters.
Using some geometry as detailed in the Q&A, the car's coordinates are translated into 'local' coordinates, rather than
map coordinates while generating the trajectory and then rotated back before returning. 
 Then, several waypoints are targeted based on the current velocity and lane (just 3) and the 
spline library is used to fill the in the rest.  After we reach 50 points, we prefer using the previous_path 
(as provided when we receive a websocket update)  so that we are only needing to generate a couple points per cycle.

