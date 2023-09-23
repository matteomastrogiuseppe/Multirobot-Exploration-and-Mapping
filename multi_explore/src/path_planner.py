import heapq
import numpy as np
from numba import njit

@njit(cache=True)
def A_STAR(grid, dist_cost_map, p1, p2, k):

    x_max, y_max = grid.shape
    
    # Initialize value function as inf, except for first point.
    value = np.full_like(grid, 1.0e10)
    value[p1[0],p1[1]] = grid[p1[0],p1[1]]

    if grid[p1[0],p1[1]] or grid[p2[0],p2[1]]: raise Exception("Start/goal is not in clear area.")

    # Matrices to store if point has been visited and which transition to be stored to retrieve path in the end.
    visited = np.zeros(grid.shape, dtype=np.bool_)
    transitions = np.zeros((x_max, y_max, 2), dtype=np.int32)
  
    # Priority queue, explore from most promising points (lower value function)
    priority_queue = []
    priority_queue.append((0., (p1[0],p1[1])))

    # Maximum of the potential field map
    max_dist_cost = np.max(dist_cost_map)

    # Possible grid transitions
    dxy = [[0,1], [1,0], [-1,0], [0,-1], [1,1], [1,-1], [-1,1], [-1,-1]]
    
    while priority_queue:
        # Analyze grid point with lower value function --> take current pixels
        cur_cost, (cur_x, cur_y) = heapq.heappop(priority_queue)
        
        # Break if reached goal
        if cur_x == p2[0] and cur_y == p2[1]: break
        
        # Do not consider point if it has already been visited.
        if visited[cur_x, cur_y]: continue
        
        # Analyze all possible transitions
        for dx, dy in dxy:

            # New position in grid
            x, y = cur_x + dx, cur_y + dy
            # Travelling cost different for horiz/vert transitions and diagonal ones
            if abs(x)+abs(y)==2: L=np.sqrt(2)
            else: L=1
            # Don't consider if out of bounds
            if x < 0 or x >= x_max or y < 0 or y >= y_max: continue    
            # Don't consider if obstacle            
            if grid[x,y]: continue

            # Don't consider if already visited
            if ~visited[x,y]:

                # Only consider this point if it decreases the value function from the original point
                if value[cur_x, cur_y] + L < value[x,y]:       
                    
                    value[x,y] = value[cur_x,cur_y] + L 

                    # Definition of the heuristic: first consider points which get closer to the end goal and stay far from obstacles
                    cart_distance = np.sqrt( (p2[0]-x)**2 + (p2[1]-y)**2 )**2           # activate to get A*
                    dist_cost = np.exp(distcost(dist_cost_map, max_dist_cost,x,y, k=k)) # activate to get smoothed A* 
                    heuristic = value[x,y]  + cart_distance + dist_cost
                    
                    # Insert this new point in the priority queue
                    heapq.heappush(priority_queue, (heuristic, (x, y)))
                    # Store the transition to retrieve the path 
                    transitions[x,y,0] = cur_x
                    transitions[x,y,1] = cur_y

        # Mark this point as visited
        visited[cur_x,cur_y] = 1
        
    # retrieve the path. Go backwards from end goal to starting point, using the stored transitions 
    cur_x, cur_y = p2[0], p2[1]
    path = []
    path.append((cur_x,cur_y))

    step = 0
    while (cur_x, cur_y) != (p1[0], p1[1]):
        cur_x, cur_y = transitions[(cur_x, cur_y)]
        path.append((cur_x,cur_y))
        step+=1

        # To avoid getting stuck in the loop for any reason
        if step > 500:
            return None

    return path

@njit(cache=True)
def distcost(map, max, x, y, k=1):
    # FMM Potential map has larger values when far from obstacles, 
    # so use the difference with respect to the maximum value to create a cost
    cost = map[x,y]
    return k*(max - cost)
