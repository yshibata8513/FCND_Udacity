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


 Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
<code>motion_planning.py</code>  
I explain only <code>plan_path()</code>. This function has three roles: first, to read the list of obstacle locations from a csv file and create a grid map; second, to set the start and goal locations; and third, to create a route with the A-star algorithm.


<code>planning_utils.py</code> consists of following two functions and helper two data structures.I explain two functions as follow.

1. <code>create_grid(data, drone_altitude, safety_distance)</code>  
This function creates a two-dimensional grid from the given data. The size of the grid is determined by the position of obstacles and safety_distance. The size of each cell in the grid is one meter in height and width. Each cell is assigned a value of 1 if it is within safety_distance of the obstacle, 0 otherwise. The distance from the obstacle is calculated for a given drone_altitude.

2. <code>a_star(grid, h, start, goal)</code>  
This function implements A* algorithm. In which, the nodes of the graph consist of each cell in the grid, and edges are defined between spatially adjacent cells. The cost of the edges are all set to one. Also, the heuristic function is defined as the linear distance between cell and goal.



### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
I added a few lines of code to load the first line of the csv file , as shown below.

<code> 

    # read lat0, lon0 from colliders into floating point values
    def extract_lonlat(filename):
        with open(filename,"r") as f:
            for row in f:
                break
        (lat_part,lon_part) = row.split(",")
        lat = float(lat_part.split(" ")[1])
        lon = float(lon_part.split(" ")[2])
        return lon,lat
    (lon0,lat0) = extract_lonlat(filename = 'colliders.csv')
    # set home position to (lon0, lat0, 0)
    self.set_home_position(lon0, lat0, 0.0)
        
</code>


#### 2. Set your current local position
I used <code>global_to_local</code> to convert current global position to local position based on home position in at previous step.

<code>

    # convert to current local position using global_to_local()
    _local_position = global_to_local(self.global_position, (lon0,lat0,0.0))        
</code>


#### 3. Set grid start position from local position
I identified the indices of home position in the grid as <code> (-north_offset,-east_offset) </code> and added this to <code>local_position</code> to compute the indices of <code>local_position</code> in the grid. Finally I set the indices as <code> grid_start</code>.

<code>

    # Define a grid for a particular altitude and safety margin around obstacles
    grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

    print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
    # Define starting point on the grid (this is just grid center)
    # convert start  to current position rather than map center
    grid_start = (int(_local_position[0]-north_offset), int(_local_position[1]-east_offset))
</code>


#### 4. Set grid goal position from geodetic coords
First I set latlon of <code>goal</code>position.Second I calculated the indices of <code>goal_position</code> in the grid following same procedure as <code>local_position</code>.Finally I added a simple correction procedure of <code>goal_positon</code> to avoid to set <code>goal_position</code> in obstacles.

<code>

    ## Activate this part when you want to specify goal position using latlon.  
    ## If you want to set the goal position to another latlon ,please modify following two lines
    lon_goal = self.global_position[0] + 0.001
    lat_goal = self.global_position[1] + 0.001
    _grid_goal = global_to_local((lon_goal,lat_goal,TARGET_ALTITUDE),self.global_home)
    grid_goal = ( int(_grid_goal[0]-north_offset) , int(_grid_goal[1]-east_offset) )
    while(1):
        if grid[grid_goal[0],grid_goal[1]] == 0:
            break
        else:  
            grid_goal = (grid_goal[0] - 1 , grid_goal[1] - 1)
    print("grid_goal: {}".format(grid_goal))

</code>


#### 5. Modify A* to include diagonal motion (or replace A* altogether)
I updated the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2).

First I added members of <code>Action</code> corresponding to diagonal motions as follow. 

<code>

    NORTH_EAST = (1, 1, np.sqrt(2))
    NORTH_WEST = (-1, 1, np.sqrt(2))
    SOUTH_EAST = (1, -1, np.sqrt(2))
    SOUTH_WEST = (-1, -1, np.sqrt(2))

</code>
Second I add codes to <code>valid_actions</code> to judge whether each diagonal direction is valid.For example I judge <code>NORTH_EAST</code> is valid if and only if <code>EAST</code> and <code>NORTH</code> are both valid and the cell to be moved is not in a obstacle.

<code>

    ## Neast(Nnorth,Nsouth,Nwest) means  
    ## "valid_actions does not include Action.East(Nort,South,West)"
    if Neast or Nnorth or grid[x + 1, y + 1] == 1:
        valid_actions.remove(_Action.NORTH_EAST)
    if Nwest or Nnorth or grid[x - 1, y + 1] == 1:
        valid_actions.remove(_Action.NORTH_WEST)
    if Neast or Nsouth or grid[x + 1, y - 1] == 1:
        valid_actions.remove(_Action.SOUTH_EAST)
    if Nwest or Nsouth or grid[x - 1, y - 1] == 1:
        valid_actions.remove(_Action.SOUTH_WEST)
</code>


#### 6. Cull waypoints 
To decide which points I should eliminates, I used the colliniarity check method as implemeted in <code>check_coliniarity</code>.  
In <code>prune_pathpoints</code> the method start with three points near the starting point, and if it is judged to be collinear, delete the second point, make the third point a new second point, and move the third point one point to the goal side. If it is judged not to be collinear, make the second point a new first point, make the third point a new second point, and move the third point one point to the goal side , and repeat this procedure until the third point is reached.

<code>

    def check_coliniarity(x1,x2,x3,y1,y2,y3):
        det = x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2)
        return det==0

    def prune_pathpoints(path):
        _path = []
        _path.append(path[0])
        ind1 = 0
        ind2 = 1
        ind3 = 2
        while ind3<len(path):
            (x1,y1) = path[ind1][0],path[ind1][1]
            (x2,y2) = path[ind2][0],path[ind2][1]
            (x3,y3) = path[ind3][0],path[ind3][1]
            colin = check_coliniarity(x1,x2,x3,y1,y2,y3)
            if colin:
                ind2 = ind3
                ind3 += 1
            else:
                _path.append(path[ind2])
                ind1 = ind2
                ind2 = ind3
                ind3 += 1
        _path.append(path[-1])
        return _path
         
</code>

### Execute the flight
#### 1. Does it work?
It works!


  


