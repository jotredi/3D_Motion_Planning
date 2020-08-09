## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation which consists of creating a 2D grid representation of the environment and searching it using A* from a starting position defined as the current location and a goal position in the grid.

`planning_utils.py` contains `create_grid()` for creating the 2D configuration space and `a_star()` for searching a path using basic movements in a grid.

`motion_planning.py` plans a path in the `plan_path()` method and executes it sending the waypoints to the drone as well as changing between states.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

The first step in `plan_path()` is read lat0 and lon0 from the first row of `colliders.csv` which represent the latitude and longitude values of the map center position. Then, the home position of the drone is defined as these values and 0 altitude.

#### 2. Set your current local position

For setting the current local position of the drone, we just need to transform the global position into local position using `global_to_local()` (line 172 in `motion_planning.py`)

#### 3. Set grid start position from local position

The grid start position is defined as the local position of the drone inside the grid (line 197)

#### 4. Set grid goal position from geodetic coords

I set the goal position as a random location inside the grid that doesn't collide with any obstacles (lines - )

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

I have modified the code in `planning_utils.py` to account for diagonal motions with a cost of sqrt(2) in the A* implementation.

I have also added an implementation of A* for graphs in `graph_utils.py` and 3D A* for searching a path in a 3D grid in `utils_3D.py`.

In my final implementation, I define the environment as a 3D voxmap centered at the target altitude of the drone to be able to plan 3D paths. The resolution of this grid is 10m³ so this will allow a fast computation of a coarse global plan for the drone using 3D A*.

##### Receding Horizon

In order to fine tune the calculated path and be able to react to new obstacles or other uncertainties, I am continuously replanning a new path using a high resolution grid around the current position.

This is done in `plan_local_path()` inside `motion_planning.py`. Here, I define a 40x40x10 m voxmap around the current position and I plan a path from there until the next global waypoint. For doing this, I set the goal location as a limit node inside the grid in the direction of the next waypoint (line 308).

I maintain 5 local waypoints inside `self.local_waypoints` (line 72) and the drone follows these waypoints instead of the global waypoints. Using a higher number of local waypoints at each time will not account for many changes in the environment.

#### 6. Cull waypoints

Every time I generate a path, I use a collinearity check for removing unnecessary waypoints. This is implemented in `prune_path()` inside `planning_utils.py`.

### Execute the flight

The drone is able to execute the flight from starting point to a random goal location.
