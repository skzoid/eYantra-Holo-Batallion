import matplotlib.pyplot as plt
import numpy as np
import math
import heapq

def plot_the_path( bot_ids , paths):
        x = []
        y = []

        #plt.ion()
        plt.figure(figsize=(8,8))
        for bot_id,path in zip(bot_ids , paths):
            for a,b,c in path:
                x.append(-a)
                y.append(b)
        

            plt.plot(x , y , '-o',label = f"{bot_id}")
            x = []
            y = []
        plt.legend()
        plt.xlim((-61,0))
        plt.ylim((0,61))
        plt.show(block = True)
        #plt.pause(0.001)

def remove_linear_points(path):
    removed = []
    for i in range(len(path)):
            if(i == 0 or i == len(path)-1):
                  continue
                  
            prev = np.array(path[i-1])
            next = np.array(path[i+1])
            curr = np.array(path[i])

            if(np.allclose(curr - prev , next - curr , rtol=0 , atol=0.05)):
                  removed.append(i)
    
    new_path = []

    for i in range(len(path)):
          if i not in removed:
                new_path.append(path[i])
            
    return new_path





def astar_time(grid, start, goal, reserved_vertices, reserved_edges,robot_id,start_time_step=0,grid_size=40.0):
    start = (int(start[0]/grid_size), int(start[1]/grid_size))
    goal  = (int(goal[0]/grid_size),  int(goal[1]/grid_size))
    rows, cols = len(grid), len(grid[0])
    directions = [
        (-1,  0, 1), (1,  0, 1), (0, -1, 1), (0,  1, 1),
        (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),  (1,  1, math.sqrt(2)),
        (0, 0, 1)
    ]

    def heuristic(a, b):
        dy = np.abs(a[0] - b[0])
        dx = np.abs(a[1] - b[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
     
    start_state = (start[0], start[1], start_time_step) 
    

    open_list = []
    heapq.heappush(open_list, (0, start_state))
    came_from = {}
    g_cost = {start_state: 0}

    closed_set = set()

    while open_list:
        _, current = heapq.heappop(open_list)
        x,y,t = current

        if (x,y) == goal:
            return reconstruct_path(came_from, current)

        closed_set.add(current)

        for dx, dy, move_cost in directions:
            nx, ny = x + dx, y + dy
            nt = t + 1
            neighbor = (nx, ny,nt)

            if nx < 0 or nx >= rows or ny < 0 or ny >= cols:
                continue
            if grid[nx][ny] != 5:
                continue
            if (nx, ny, nt) in reserved_vertices and reserved_vertices[(nx,ny,nt)] != robot_id:
                continue
            if (nx,ny,x,y,nt) in reserved_edges or (x,y,nx,ny,nt) in reserved_edges:
                continue
            if neighbor in closed_set:
                continue

            tentative_g = g_cost[current] + move_cost

            if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                came_from[neighbor] = current
                g_cost[neighbor] = tentative_g
                f_cost = tentative_g + heuristic((nx, ny), goal)
                heapq.heappush(open_list, (f_cost, neighbor))

    return None 




def reconstruct_path(came_from, current):   
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]




def reserve_path(path,reserved_vertices,reserved_edges,robot_id):
    def f_surr(x,y,t):
        reserved_vertices[(x,y,t)] = robot_id
        for j in range(61):
            for k in range(61):
                distance = np.sqrt((x-k)**2 + (y-j)**2)
                if distance <= 6:
                    reserved_vertices[(k,j,t)] = robot_id
                    
    for i in range(len(path)):
        x,y,t = path[i]
        
        

        if (i != len(path) - 1):
             f_surr(x,y,t)
        else:
            for j in range(30):
                f_surr(x,y,t+j)
         
        if i>0:
            px,py,pt = path[i-1]
            reserved_edges[(x,y,px,py,t)] = robot_id





    