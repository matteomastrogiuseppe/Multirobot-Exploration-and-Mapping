import heapq
import numpy as np
from numba import njit
import matplotlib.pyplot as plt
import pyfmm
from time import time 

@njit(cache=True)
def A_STAR(grid, dist_cost_map, p1, p2, k):

    x_max, y_max = grid.shape
    value = np.full_like(grid, 1.0e10)
    value[p1[0],p1[1]] = grid[p1[0],p1[1]]

    if grid[p1[0],p1[1]] : raise Exception("Start/goal is not in clear area.") #or  grid[p2[0], p2[1]]

    visited = np.zeros(grid.shape, dtype=np.bool_)
    transitions = np.zeros((x_max, y_max, 2), dtype=np.int32)
  
    priority_queue = []
    priority_queue.append((0., (p1[0],p1[1])))

    x_max, y_max = grid.shape
    max_dist_cost = np.max(dist_cost_map)

    dxy = [[0,1], [1,0], [-1,0], [0,-1], [1,1], [1,-1], [-1,1], [-1,-1]]
    
    while priority_queue:
        cur_cost, (cur_x, cur_y) = heapq.heappop(priority_queue)
        
        if cur_x == p2[0] and cur_y == p2[1]:
            break
        
        if visited[cur_x, cur_y]:
            continue
        
        for dx, dy in dxy:
            x, y = cur_x + dx, cur_y + dy
            if abs(x)+abs(y)==2: L=np.sqrt(2)
            else: L=1
            if x < 0 or x >= x_max or y < 0 or y >= y_max:
                continue                
            if grid[x,y]: continue
            if ~visited[x,y]:
                if value[cur_x, cur_y] + L < value[x,y]:       
                    
                    value[x,y] = value[cur_x,cur_y] + L 
                    cart_distance = np.sqrt( (p2[0]-x)**2 + (p2[1]-y)**2 ) # activate to get A*
                    dist_cost = distcost(dist_cost_map, max_dist_cost,x,y, p1,k=k) # activate to get smoothed A* 
                    heuristic = value[x,y]  + cart_distance + dist_cost
                    heapq.heappush(priority_queue, (heuristic, (x, y)))
                    transitions[x,y,0] = cur_x
                    transitions[x,y,1] = cur_y

        visited[cur_x,cur_y] = 1
        
    # retrieve the path
    cur_x, cur_y = p2[0], p2[1]
    path = []
    path.append((cur_x,cur_y))

    while (cur_x, cur_y) != (p1[0], p1[1]):
        cur_x, cur_y = transitions[(cur_x, cur_y)]
        path.append((cur_x,cur_y))

    return path

@njit(cache=True)
def distcost(map, max, x, y, p, k=1):
    cost = map[x,y]
    return k*(max - cost)

def get_xy_vision(grid, p, vision):

    xv_min = max(p[0]-vision, 0)
    yv_min = max(p[1]-vision, 0)
    xv_max = min(p[0]+vision, grid.shape[0])
    yv_max = min(p[1]+vision, grid.shape[1])

    return (xv_min, xv_max), (yv_min, yv_max)