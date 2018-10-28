## Project: 3D Motion Planning
![Quad Image](/misc/enroute.png)

---

## Rubric Points
### Here I will consider the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points individually and describe how I addressed each point in my implementation.

---

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
The abovementioned scripts contain a basic planning implementation that includes a state based control loop for managing the drones behaviour, aswell as planning a suitable path.  The functionality in `planning_utils.py` allows the world to be discretised into a grid or graph representation, while the `motion_planning.py` code is an implementation of the Drone object for controlling the behaviour and mission of the drone.

An overview of the two solutions provided are as follows:

Feature | backyard_flyer | motion_planning
--- | --- | ---
`Implements a path planner for finding paths` | no | **yes**
`Uses dynamic searching for missions` | no | **yes**
`Includes an internal state of the configuration space` | no | **yes**
`Uses global positioning for following paths` | no | **yes**

### Implementing Your Path Planning Algorithm
#### On the A* Search using Potential Fields based algorithm...

#### 1. Set your global home position
To accurately generate a valid path that the quadcopter can follow, it needs to know the relative coordinates of each successive waypoint.  This is accomplished by setting the home location, which in this case is obtained by reading the provided configuration space state in the 'colliders' file.

``` 
with open(filename) as f:
            lat, lon = [float(x.replace("lat0 ", "").replace("lon0 ", "")) for x in f.readline().split(',')]
```

#### 2. Set your current local position
After the global coordinates have been obtained, the quadcopter needs to realise the home location for advancing to waypoints relative to its position in the world.  This is achieved by the following line:

```
global_home = np.array([lon, lat, 0.0])            

```

#### 3. Set grid start position from local position
When constructing the grid representation, the world is discretized to reduce the size of the grid space so that the minimum data points form the upper left of the grid.  This means that the relative center is the offset of each direction in the NED frame.

Here we obtain the offsets for determining our global position relative to the grid world:
```
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        
grid_start = (int(current_local_position[0] - north_offset), int(current_local_position[1]- east_offset))
```

#### 4. Set grid goal position from geodetic coords
Once the relative offsets are determined, the actual local position is taken using the global GNSS and home coordinates:

```
goal_lon, goal_lat = -122.395741, 37.797475
goal_global_position = np.array([goal_lon,goal_lat,0])
goal_local_position = global_to_local(goal_global_position,global_home)
grid_goal = (int(goal_local_position[0] - north_offset),
            int(goal_local_position[1] - east_offset))
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
In this project, A* procedure are modified to include diagonal motion.

> Path with diagonal motion:  
![Path with diagonal motion](/images/diagonal_motion.png)


#### 6. Prune waypoints 
To remove redundant waypoints, I have tried two path expansion algorithms. The first algorithm is collinearity test, and the second uses the  bresenham for iteratively finding the maximal visible waypoint from each candidate state. 

The results below compare the results of the original path, collineartiy path and bresenham path. Both algorithms can remove redundant waypoints. And Bresenham has a better result than collinearity. 

Feature | original | collinearity | bresenham
--- | --- | --- | ---
`WaypointsNum` | 631 | 44 | 28

The pictures below explain why bresenham is better. Since diagnal motion is introduced, for example, when the original path repeatly take actions Action.UP_RIGHT and Action.RIGHT for a long distance, Collinearity test will fail to remove many waypoints. But Bresenham can solve this problem.
> Original path  
![Original path](/images/diagonal_motion.png)
> Collinearity path   
![Collinearity Path](/images/collinearity_path.png)    
> Bresenham path   
![Bresenham Path](/images/bres_path.png)

### Execute the flight
#### 1. Does it work?
It works!

![In action](/Images/in-action.png)


