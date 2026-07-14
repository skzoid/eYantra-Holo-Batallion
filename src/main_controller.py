#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import heapq
from hb_interfaces.msg import Pose2D, Poses2D,BotCmd ,BotCmdArray
from std_srvs.srv import SetBool
from PID_test_1 import HolonomicPIDController
from rclpy.executors import SingleThreadedExecutor
from acc import plot_the_path,remove_linear_points,astar_time,reconstruct_path,reserve_path


class fsm(Node):
    def __init__(self , exec):
        super().__init__("fsm_node")

        self.executor = exec

        self.num_crates = 4
        self.num_robots = 3

        self.robots = {}
        self.tasks = {}

        self.assignments = {}
        self.remaining_tasks = set()
        self.task_lock = False
               
        self.pose_sub = self.create_subscription(Poses2D , '/bot_pose' , self.pose_cb , 10)
        self.crate_sub = self.create_subscription(Poses2D , '/crate_pose' , self.crate_cb , 10)
        self.publisher = self.create_publisher(BotCmdArray, '/bot_cmd', 100)
    

        self.attach_client_0 = self.create_client(SetBool , '/attach/bot0')
        self.attach_client_2 = self.create_client(SetBool , '/attach/bot2')
        self.attach_client_4 = self.create_client(SetBool , '/attach/bot4')


        self.obstacle_grid_0 = np.ones(shape = (61,61))*5
        self.obstacle_grid_2 = np.ones(shape = (61,61))*5
        self.obstacle_grid_4 = np.ones(shape = (61,61))*5

        self.robot_0_planned = False
        self.robot_2_planned = False
        self.robot_4_planned = False

        self.task_allocated = False
    
        self.robot_0_goals = None
        self.robot_2_goals = None
        self.robot_4_goals = None
        self.goals_set = {}

        self.controller_0 = None
        self.controller_2 = None
        self.controller_4 = None

        self.controller_0_goals_reached = False
        self.controller_2_goals_reached = False
        self.controller_4_goals_reached = False

        self.states_order = ['IDLE' , 'MOVING' , 'PICKUP' , 'DROP_PLANNING' , 'MOVING' , 'DROP', 'LOOK_FOR_NEXT' , 'DOCK_PLANNING' , 'MOVING' ]
        #self.states_order = ['IDLE' , 'MOVING' , 'DROP_PLANNING' , 'MOVING' , 'LOOK_FOR_NEXT' , 'DOCK_PLANNING' , 'MOVING' ]
        self.state_idx_0 = 0
        self.state_idx_2 = 0
        self.state_idx_4 = 0

        self.D1 = [1215 , 1215]
        self.D2 = [820 , 2017.5]
        self.D3 = [1616 , 2017.5]

        self.D1_bookings = {0:[[1215,1215],[1221,1215],[1209,1215],[1227,1215],[1203,1225]] , 1:[[1218,1215],[1224,1215],[1212,1215],[1224,1215]]}
        self.D2_bookings = {0:[[820,2017.5],[826,2017.5],[814,2017.5],[832,2017.5],[806,2017.5]] , 1:[[823,2017.5],[817,2017.5],[829,2017.5],[811,2017.5]]}
        self.D3_bookings = {0:[[1616,2017.5],[1622,2017.5],[1610,2017.5],[1628,2017.5],[1604,2017.5]] , 1:[[1619,2017.5],[1613,2017.5],[1625,2017.5],[1607,2017.5]]}
        self.num_D1 = 0
        self.num_D2 = 0
        self.num_D3 = 0
        self.D1_idx = 0
        self.D2_idx = 0
        self.D3_idx = 0
        self.count_allocated = False
        self.drop_allocated = {}

        

        self.dock_0 = [1218 , 205]
        self.dock_2 = [1568 , 202]
        self.dock_4 = [864 , 204]

        



        self.time_0_1 = None
        self.time_0_2 = None

        self.time_2_1 = None
        self.time_2_2 = None

        self.time_4_1 = None
        self.time_4_2 = None

        self.stop_and_go_flag = False
        self.grid_size = 40

        self.time_step = 0


        self.bots_in_radius = []

        for client in [self.attach_client_0, self.attach_client_2, self.attach_client_4]:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn("Attach service not available")

            


        self.timer_0 = self.create_timer(0.1 , self.update_0)
        self.timer_2 = self.create_timer(0.1 , self.update_2)
        self.timer_4 = self.create_timer(0.1 , self.update_4)
        self.timer = self.create_timer(0.01 , self.update_time_step)
        #self.collision_checker_timer = self.create_timer(0.1 , self.collision_checker)
        #self.stop_and_go_timer = self.create_timer(0.1 , self.stop_and_go)

        self.reserved_edges = {}
        self.reserved_vertices = {}



    def send_attach_request(self, bot_id):
        request = SetBool.Request()
        request.data = True
        if(bot_id == 0):
            future = self.attach_client_0.call_async(request)
        elif(bot_id == 2):
            future = self.attach_client_2.call_async(request)
        elif(bot_id == 4):
            future = self.attach_client_4.call_async(request)
        

        future.add_done_callback(self.attach_done_callback)
        


    
    def attach_done_callback(self , future):
        response = future.result()
        print(response)
    
    def arm_control(self , bot_id ,base = 90, elbow = 90):
        msg = BotCmd()
        msg.m1 = np.float64(90)
        msg.m2 = np.float64(90)
        msg.m3 = np.float64(90)
        msg.id = bot_id
        msg.elbow = np.float64(elbow)
        msg.base = np.float64(base)
        Msg = BotCmdArray()
        Msg.cmds = [msg]
        self.publisher.publish(Msg)




    def send_detach_request(self, bot_id):
        request = SetBool.Request()
        request.data = False
        if(bot_id == 0):
            future = self.attach_client_0.call_async(request)
        elif(bot_id == 2):
            future = self.attach_client_2.call_async(request)
        elif(bot_id == 4):
            future = self.attach_client_4.call_async(request)
        future.add_done_callback(self.detach_done_callback)

    def detach_done_callback(self , future):
        response = future.result()
        print(response)

    def update_grid(self ,grid ,bot_id, grid_size = 40 ):

        crate_offsets = [0]
        bot_offsets = [0]
        for id,pose in self.tasks.items():
            if(id != self.assignments[bot_id]):
                x = int(pose[0]/grid_size)
                y = int(pose[1]/grid_size)
                for offset_x in crate_offsets:
                    for offset_y in crate_offsets:
                        if(0 <= x+offset_x < 61 and 0 <= y + offset_y < 61 ):
                            grid[x+offset_x][y+offset_y] = 1

        for id,pose in self.robots.items():
            if(id != bot_id):
                x = int(pose[0]/grid_size)
                y = int(pose[1]/grid_size)
                for offset_x in bot_offsets:
                    for offset_y in bot_offsets:
                        if(0 <= x+offset_x < 61 and 0 <= y + offset_y < 61 ):
                            grid[x+offset_x][offset_y+y] = id

        pts = []
        c1 = np.array([1020.0,1355.0])
        c2 = np.array([1410.0,1355.0])
        c3 = np.array([1410.0,1075.0])
        c4 = np.array([1020.0,1075.0])

        ms = np.linspace(0,1,15)

        l1 = []
        for m in ms:
            pt = c1 * m + c2 * (1-m)
            l1.append(pt)

        l2 = []
        for m in ms:
            pt = c2 * m + c3 * (1-m)
            l2.append(pt)

        l3 = []
        for m in ms:
            pt = c1 * m + c4 * (1-m)
            l3.append(pt)

        pts = l1 + l2 + l3
        self.add_dock_obstacles(grid , pts , grid_size)
    
    def allocate_drop_location(self):

        if self.count_allocated is False:

            self.count_allocated = True
            for id,pos in self.tasks.items():
                if id%3 == 0:
                    self.num_D1 += 1
                if id%3 == 1:
                    self.num_D2 += 1
                if id%3 == 2:
                    self.num_D3 += 1
            


        
        for _,crate_id in self.assignments.items():
            if crate_id not in self.drop_allocated.keys():
                if crate_id % 3 == 0:
                    if self.D1_idx <= self.num_D1/2:
                        self.drop_allocated[crate_id] = [0,self.D1_bookings[0][self.D1_idx]]
                        self.D1_idx += 1
                    else:
                        self.drop_allocated[crate_id] = [1,self.D1_bookings[1][self.D1_idx] - self.num_D1/2]
                        self.D1_idx += 1
                if crate_id % 3 == 1:
                    if self.D2_idx <= self.num_D2/2:
                        self.drop_allocated[crate_id] = [0,self.D2_bookings[0][self.D2_idx]]
                        self.D2_idx += 1
                    else:
                        self.drop_allocated[crate_id] = [1,self.D2_bookings[1][self.D2_idx] - self.num_D2/2]
                        self.D2_idx += 1
                if crate_id % 3 == 2:
                    if self.D3_idx <= self.num_D3/2:
                        self.drop_allocated[crate_id] = [0,self.D3_bookings[0][self.D3_idx]]
                        self.D3_idx += 1
                    else:
                        self.drop_allocated[crate_id] = [1,self.D3_bookings[1][self.D3_idx] - self.num_D3/2]
                        self.D3_idx += 1
                




            




    def add_dock_obstacles(self , grid , pts , grid_size = 40):
        for pt in pts:
            x = int(pt[0]//grid_size)
            y = int(pt[1]//grid_size)

            grid[x][y] = 1.0

                
                        

    



    def pose_cb(self, msg):

        for pose in msg.poses:
                yaw_in_degrees = pose.w
                yaw_in_radians = math.radians(yaw_in_degrees)
                self.robots[pose.id] = [pose.x, pose.y, yaw_in_radians]

                
                
            
    def crate_cb(self , msg):

        for pose in msg.poses:
            if pose.id not in [1,3,5,7]:
                yaw_in_degrees = pose.w
                yaw_in_radians = math.radians(yaw_in_degrees)

                self.tasks[pose.id] = [pose.x, pose.y, yaw_in_radians]
    

    def cost_function(self , robot_position , task_position):
        dy = np.abs(robot_position[0]-task_position[0])
        dx = np.abs(robot_position[1]-task_position[1])
        cost = np.sqrt(dx**2 + dy**2)
        return cost
    
    def calculate_pickup_pose(self,x,y,theta,d):
            
            p1 = np.array([x,y]) + np.array([d*np.cos(theta),d*np.sin(theta)])
            p2 = np.array([x,y]) + np.array([-d*np.sin(theta),d*np.cos(theta)])
            p3 = np.array([x,y]) + np.array([-d*np.cos(theta),-d*np.sin(theta)])
            p4 = np.array([x,y]) + np.array([d*np.sin(theta),-d*np.cos(theta)])

            pos = [p1,p2,p3,p4]
            pos_x = [p1[0],p2[0],p3[0],p4[0]]
            

            if(x < 1200.0):
                idx = np.argmax(pos_x)
                return pos[idx][0],pos[idx][1]
            else:
                idx = np.argmin(pos_x)
            
                return pos[idx][0],pos[idx][1]
    
    def greedy_allocation(self,robots,tasks):
        assignmnets={}
        remaining_robots = set(robots.keys())
        remaining_tasks = set(tasks.keys())
        while remaining_robots and remaining_tasks:
            min_cost = None
            best_pair = None
            
            for r_id in remaining_robots:
                for t_id in remaining_tasks:
                    cost = self.cost_function(robots[r_id],tasks[t_id])
                    if min_cost is None or cost <min_cost:
                        min_cost = cost
                        best_pair = (r_id,t_id)
        
            r_id,t_id = best_pair
            assignmnets[r_id]=t_id
            remaining_robots.remove(r_id)
            remaining_tasks.remove(t_id)
        self.remaining_tasks = remaining_tasks
        return assignmnets 
    

        
    def astar(self , grid, start, goal , grid_size = 40):
        start = (int(start[0]/grid_size), int(start[1]/grid_size))
        goal  = (int(goal[0]/grid_size),  int(goal[1]/grid_size))

        open_list = []
        heapq.heappush(open_list, (0, start))
        rows, cols = len(grid), len(grid[0])
        directions = [
            (-1,  0, 1), (1,  0, 1), (0, -1, 1), (0,  1, 1),
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),  (1,  1, math.sqrt(2))
        ]

        def heuristic(a, b):
            dy = np.abs(a[0] - b[0])
            dx = np.abs(a[1] - b[1])
            return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)




        came_from = {}
        g_cost = {start: 0}

        closed_set = set()

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            closed_set.add(current)

            for dx, dy, move_cost in directions:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)

                if nx < 0 or nx >= rows or ny < 0 or ny >= cols:
                    continue
                if grid[nx][ny] == 1:
                    continue
                if neighbor in closed_set:
                    continue

                tentative_g = g_cost[current] + move_cost

                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_g
                    f_cost = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_cost, neighbor))

        return None 
    
    def reconstruct_path(self, came_from , current):   
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
    



    

    def collision_checker(self , safety_radius = 300.0):
        # if(self.robot_0_goals is not None and self.robot_2_goals is not None and self.robot_4_goals is not None):
        #     plot_the_path(bot_ids=[0,2,4] , paths=[self.robot_0_goals,self.robot_2_goals,self.robot_4_goals])

        if(len(self.robots) < 3):
            return
        if(self.cost_function(self.robots[0] , self.robots[2])<safety_radius):
            
            
            if(self.controller_0 is None and self.controller_2 is not None):
                if(self.state_idx_0 < len(self.states_order)):
                    self.controller_2.pause()

            elif(self.controller_2 is None and self.controller_0 is not None):
                if(self.state_idx_2 < len(self.states_order)):
                    self.controller_0.pause()
                
            elif(self.controller_2 is None and self.controller_0 is None):
                pass
            elif(0 not in self.bots_in_radius or 2 not in self.bots_in_radius):

                self.controller_0.pause()
                self.controller_2.pause()


                if(0 not in self.bots_in_radius and 2 not in self.bots_in_radius):
                    dist_from_goal_0 = self.cost_function(self.robots[0][:2] , self.robot_0_goals[-1])
                    dist_from_goal_2 = self.cost_function(self.robots[2][:2] , self.robot_2_goals[-1])
                    if(dist_from_goal_0 < dist_from_goal_2):
                        self.bots_in_radius.append(0)
                        print("-"*10,"0 Appended!")
                        self.bots_in_radius.append(2)
                        print("-"*10,"2 Appended!")
                    elif(dist_from_goal_0 > dist_from_goal_2):
                        self.bots_in_radius.append(2)
                        print("-"*10,"2 Appended!")
                        self.bots_in_radius.append(0)
                        print("-"*10,"0 Appended!")

                elif 0 in self.bots_in_radius and 2 not in self.bots_in_radius:
                    self.bots_in_radius.append(2)
                    print("-"*10,"2 Appended!")
                
                elif 2 in self.bots_in_radius and 0 not in self.bots_in_radius:
                    self.bots_in_radius.append(0)
                    print("-"*10,"0 Appended!")


        if(self.cost_function(self.robots[2] , self.robots[4])<safety_radius):

            if(self.controller_2 is None and self.controller_4 is not None):
                if(self.state_idx_2 < len(self.states_order)):
                    self.controller_4.pause()
                

            elif(self.controller_4 is None and self.controller_2 is not None):
                if(self.state_idx_4 < len(self.states_order)):
                    self.controller_2.pause()
                
            elif(self.controller_2 is None and self.controller_4 is None):
                pass
            elif 2 not in self.bots_in_radius or 4 not in self.bots_in_radius:

                self.controller_2.pause()
                self.controller_4.pause()


                if(2 not in self.bots_in_radius and 4 not in self.bots_in_radius):
                    dist_from_goal_2 = self.cost_function(self.robots[2][:2] , self.robot_2_goals[-1])
                    dist_from_goal_4 = self.cost_function(self.robots[4][:2] , self.robot_4_goals[-1])
                    if(dist_from_goal_2 < dist_from_goal_4):
                        self.bots_in_radius.append(2)
                        print("-"*10,"2 Appended!")
                        self.bots_in_radius.append(4)
                        print("-"*10,"4 Appended!")
                    elif(dist_from_goal_2 > dist_from_goal_4):
                        self.bots_in_radius.append(4)
                        print("-"*10,"4 Appended!")
                        self.bots_in_radius.append(2)
                        print("-"*10,"2 Appended!")

                elif 2 in self.bots_in_radius and 4 not in self.bots_in_radius:
                    self.bots_in_radius.append(4)
                    print("-"*10,"4 Appended!")
                
                elif 4 in self.bots_in_radius and 2 not in self.bots_in_radius:
                    self.bots_in_radius.append(2)
                    print("-"*10,"2 Appended!")


        if((cost := self.cost_function(self.robots[4] , self.robots[0]))<safety_radius):
            print("Dist : ", cost)
            if(self.controller_0 is None and self.controller_4 is not None):
                if(self.state_idx_0 < len(self.states_order)):
                    self.controller_4.pause()
                

            elif(self.controller_4 is None and self.controller_0 is not None):
                if(self.state_idx_4 < len(self.states_order)):
                    self.controller_0.pause()
                
            elif(self.controller_0 is None and self.controller_4 is None):
                pass
            elif 0 not in self.bots_in_radius or 4 not in self.bots_in_radius :

                self.controller_0.pause()
                self.controller_4.pause()

                if(0 not in self.bots_in_radius and 4 not in self.bots_in_radius):
                    dist_from_goal_0 = self.cost_function(self.robots[0][:2] , self.robot_0_goals[-1])
                    dist_from_goal_4 = self.cost_function(self.robots[4][:2] , self.robot_4_goals[-1])
                    if(dist_from_goal_0 < dist_from_goal_4):
                        self.bots_in_radius.append(0)
                        print("-"*10,"0 Appended!")
                        self.bots_in_radius.append(4)
                        print("-"*10,"4 Appended!")
                    elif(dist_from_goal_0 > dist_from_goal_4):
                        self.bots_in_radius.append(4)
                        print("-"*10,"4 Appended!")
                        self.bots_in_radius.append(0)
                        print("-"*10,"0 Appended!")

                elif 0 in self.bots_in_radius and 4 not in self.bots_in_radius:
                    self.bots_in_radius.append(4)
                    print("-"*10,"4 Appended!")
                
                elif 4 in self.bots_in_radius and 0 not in self.bots_in_radius:
                    self.bots_in_radius.append(0)
                    print("-"*10,"0 Appended!")
                

    def stop_and_go(self , safety_radius = 400.0):
        if(len(self.bots_in_radius) == 0):
            return
           
        if(len(self.bots_in_radius) == 1):
            if(self.bots_in_radius[0] == 0):
                self.controller_0.resume()
            if(self.bots_in_radius[0] == 2):
                self.controller_2.resume()
            if(self.bots_in_radius[0] == 4):
                self.controller_4.resume()
            self.bots_in_radius.pop(0)

        else:
            if(self.bots_in_radius[0] == 0 and self.state_idx_0 >= len(self.states_order)):
                self.bots_in_radius.pop(0)
                return
            if(self.bots_in_radius[0] == 2 and self.state_idx_2 >= len(self.states_order)):
                self.bots_in_radius.pop(0)
                return
            if(self.bots_in_radius[0] == 4 and self.state_idx_4 >= len(self.states_order)):
                self.bots_in_radius.pop(0)
                return
            
            if(self.cost_function(self.robots[self.bots_in_radius[0]] , self.robots[self.bots_in_radius[1]] ) > safety_radius ):
                self.stop_and_go_flag = False
                print("-"*10,f"Removed {self.bots_in_radius[0]}")
                self.bots_in_radius.pop(0)
                return
            if(self.stop_and_go_flag is False):
                self.stop_and_go_flag = True
                if(self.bots_in_radius[0] == 0):
                    self.executor.remove_node(self.controller_0)
                    self.controller_0.destroy_node()
                    self.controller_0 = None
                    self.controller_0_goals_reached = True
                    self.state_idx_0 -= 1
                elif(self.bots_in_radius[0] == 2):
                    self.executor.remove_node(self.controller_2)
                    self.controller_2.destroy_node()
                    self.controller_2 = None
                    self.controller_2_goals_reached = True
                    self.state_idx_2 -= 1
                elif(self.bots_in_radius[0] == 4):
                    self.executor.remove_node(self.controller_4)
                    self.controller_4.destroy_node()
                    self.controller_4 = None
                    self.controller_4_goals_reached = True
                    self.state_idx_4 -= 1
            else:
                return
                

        

    
    

    def update_0(self , bot_id = 0):
        if(self.state_idx_0 >= len(self.states_order)):
            self.get_logger().info("Mission Accomplished !")
            self.arm_control(bot_id=bot_id)
            return
        elif(self.states_order[self.state_idx_0] == 'IDLE'):
            if(len(self.robots) == self.num_robots and len(self.tasks) == self.num_crates):
                if(self.task_allocated is False):
                    self.assignments = self.greedy_allocation(self.robots , self.tasks)
                    print(self.assignments)
                    self.task_allocated = True
                    self.allocate_drop_location()
                elif(self.robot_0_planned is False):    
                    self.obstacle_grid_0  = np.zeros(shape=(61,61))
                    self.update_grid(self.obstacle_grid_0 , bot_id)

                    if(self.controller_2 is not None and self.controller_2.return_time_step() != -1):
                        self.time_step = self.controller_2.return_time_step()
                    elif(self.controller_4 is not None and self.controller_4.return_time_step() != -1):
                        self.time_step = self.controller_4.return_time_step()
                    else:
                        self.time_step = 0
                    print(f"{bot_id} : {self.time_step}")
                    tx,ty = self.calculate_pickup_pose(self.tasks[self.assignments[bot_id]][0],self.tasks[self.assignments[bot_id]][1],self.tasks[self.assignments[bot_id]][2],120.0)
                    self.robot_0_goals = astar_time(self.obstacle_grid_0 , self.robots[bot_id][:2] , [tx,ty] ,self.reserved_vertices,self.reserved_vertices,start_time_step=self.time_step)
                    

                    
                    
                    
                    if(self.robot_0_goals is not None):
                        
                        #self.robot_0_goals = remove_linear_points(self.robot_0_goals)
                        #plot_the_path([bot_id] , [self.robot_0_goals])
                        reserve_path(self.robot_0_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                        self.robot_0_goals[-1] = (tx,ty,self.robot_0_goals[-1][2])
                        print(f"Bot : {bot_id} \n Goals  {self.robot_0_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                        self.robot_0_planned = True
                else:
                    self.state_idx_0 += 1
                    self.robot_0_planned = False
            else:
                print("Not enough poses.")
        
        elif(self.states_order[self.state_idx_0] == 'MOVING'):
            if(self.controller_0 is None and self.robot_0_goals is not None):

                if(self.state_idx_0 == 1):
                    self.controller_0 = HolonomicPIDController(bot_id , 20 , False , self.assignments[0])
                elif(self.state_idx_0 == 4):
                    self.controller_0 = HolonomicPIDController(bot_id , 20 )
                else:
                    self.controller_0 = HolonomicPIDController(bot_id )
                self.executor.add_node(self.controller_0)
                world_goals = []
                if(len(self.robot_0_goals) >= 2):
                    self.robot_0_goals.pop(0)
                for goal in self.robot_0_goals:
                    world_goals.append((goal[0]*40 + 20 , goal[1]*40 + 20 , goal[2]))
                if(self.state_idx_0 == 1):
                    world_goals[-1] = (world_goals[-1][0] - 20.0)/40.0,(world_goals[-1][1] - 20.0)/40.0,world_goals[-1][2]
                self.controller_0.set_goals(world_goals)


            elif(self.controller_0 is None and self.controller_0_goals_reached is True):

                self.executor.remove_node(self.controller_0)
                self.controller_0.destroy_node()
                self.controller_0 = None
                self.controller_0_goals_reached = False
                self.state_idx_0 += 1
            elif(self.controller_0 is not None and self.controller_0.all_goals_reached is True):

                self.executor.remove_node(self.controller_0)
                self.controller_0.destroy_node()
                self.controller_0 = None
                self.controller_0_goals_reached = False
                self.state_idx_0 += 1



            
        elif(self.states_order[self.state_idx_0] == 'DROP_PLANNING'):
            drop_zone = self.drop_allocated[self.assignments[bot_id]]

            if(self.robot_0_planned is False):
                self.obstacle_grid_0 = np.zeros(shape = (61,61))
                self.update_grid(self.obstacle_grid_0 , bot_id)
                if(self.controller_2 is not None and self.controller_2.return_time_step() != -1):
                    self.time_step = self.controller_2.return_time_step()
                elif(self.controller_4 is not None and self.controller_4.return_time_step() != -1):
                    self.time_step += self.controller_4.return_time_step()
                else:
                    self.time_step = 0
                print(f"{bot_id} : {self.time_step}")
                self.robot_0_goals = astar_time(self.obstacle_grid_0 , self.robots[bot_id][:2] , drop_zone ,self.reserved_vertices,self.reserved_vertices,start_time_step=self.time_step)
                if self.robot_0_goals is not None:
                    reserve_path(self.robot_0_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                    print(f"Bot : {bot_id} \n Goals  {self.robot_0_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices}")
                    #self.robot_0_goals = remove_linear_points(self.robot_0_goals)
                    #plot_the_path([bot_id] , [self.robot_0_goals])
                    self.robot_0_planned = True

            else:
                self.state_idx_0 += 1
                self.robot_0_planned = False


        elif(self.states_order[self.state_idx_0] == 'DOCK_PLANNING'):
            if(self.robot_0_planned is False):
                self.obstacle_grid_0 = np.zeros(shape = (61,61))
                self.update_grid(self.obstacle_grid_0 , bot_id)
                #self.robot_0_goals = self.astar(self.obstacle_grid_0 , self.robots[bot_id][:2] , self.dock_0)
                
                if(self.controller_2 is not None and self.controller_2.return_time_step() != -1):
                    self.time_step += self.controller_2.return_time_step()
                elif(self.controller_4 is not None and self.controller_4.return_time_step() != -1):
                    self.time_step += self.controller_4.return_time_step()
                else:
                    self.time_step = 0
                print(f"{bot_id} : {self.time_step}")
                self.robot_0_goals = astar_time(self.obstacle_grid_0 , self.robots[bot_id][:2] , self.dock_0 ,self.reserved_vertices,self.reserved_vertices,start_time_step=self.time_step)
                if self.robot_0_goals is not None:
                    reserve_path(self.robot_0_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                    print(f"Bot : {bot_id} \n Goals  {self.robot_0_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                    #self.robot_0_goals = remove_linear_points(self.robot_0_goals)

                    #plot_the_path([bot_id] , [self.robot_0_goals])

                    self.robot_0_planned = True

            else:
                self.state_idx_0 += 1
                self.robot_0_planned = False

        elif(self.states_order[self.state_idx_0] == 'PICKUP'):
            if(self.time_0_1 is None):
                self.time_0_1 = self.get_clock().now().nanoseconds
                self.arm_control(bot_id , 7 , 90)

            elif(self.get_clock().now().nanoseconds - self.time_0_1 > 0.5e9 and self.time_0_2 is None):
                self.time_0_2 = self.get_clock().now().nanoseconds
                self.send_attach_request(bot_id = 0)
            
            elif(self.time_0_2 is not None and self.get_clock().now().nanoseconds - self.time_0_2 > 0.5e9):
                self.arm_control(bot_id, 30 , 90)
                self.state_idx_0 += 1
                self.time_0_1 = None
                self.time_0_2 = None

        elif(self.states_order[self.state_idx_0] == 'DROP'):
            if(self.time_0_1 is None):
                self.time_0_1 = self.get_clock().now().nanoseconds
                self.arm_control(bot_id , 20 , 90)

            elif(self.get_clock().now().nanoseconds - self.time_0_1 > 0.5e9 and self.time_0_2 is None):
                self.time_0_2 = self.get_clock().now().nanoseconds
                self.send_detach_request(bot_id = 0)
            
            elif(self.time_0_2 is not None and self.get_clock().now().nanoseconds - self.time_0_2 > 0.5e9):
                self.arm_control(bot_id , 130 , 90)
                self.state_idx_0 += 1
                self.time_0_1 = None
                self.time_0_2 = None
        elif(self.states_order[self.state_idx_0] == 'LOOK_FOR_NEXT'):
            if(self.task_lock is False):
                self.task_lock = True
                if len(self.remaining_tasks) != 0:
                    task_assigned = None
                    min_dist = None
                    for task in self.remaining_tasks:
                        dist = self.cost_function(self.robots[bot_id][:2] ,self.tasks[task][:2])
                        if min_dist is None or dist < min_dist:
                            min_dist = dist
                            task_assigned = task
                    self.assignments[bot_id] = task_assigned
                    self.state_idx_0 = 0
                    self.remaining_tasks.remove(task_assigned)

                else:
                    self.state_idx_0 += 1

                self.task_lock = False

        

    def update_2(self , bot_id = 2):


        if(self.state_idx_2 >= len(self.states_order)):
            self.get_logger().info("Mission Accomplished !")
            self.arm_control(bot_id=2)
            return
        elif(self.states_order[self.state_idx_2] == 'IDLE'):
            if(len(self.robots) == self.num_robots and len(self.tasks) == self.num_crates):
                if(self.task_allocated is False):
                    self.assignments = self.greedy_allocation(self.robots , self.tasks)
                    print(self.assignments)
                    self.task_allocated = True
                elif(self.robot_2_planned is False):    
                    self.obstacle_grid_2  = np.zeros(shape=(61,61))
                    self.update_grid(self.obstacle_grid_2 , bot_id)
                    #self.robot_2_goals = self.astar(self.obstacle_grid_2 , self.robots[bot_id][:2] , self.tasks[self.assignments[bot_id]][:2])
                    if(self.controller_0 is not None and self.controller_0.return_time_step() != -1):
                        self.time_step = self.controller_0.return_time_step()
                    elif(self.controller_4 is not None and self.controller_4.return_time_step() != -1):
                        self.time_step = self.controller_4.return_time_step()
                    else:
                        self.time_step = 0
                    print(f"{bot_id} : {self.time_step}")
                    tx,ty = self.calculate_pickup_pose(self.tasks[self.assignments[bot_id]][0],self.tasks[self.assignments[bot_id]][1],self.tasks[self.assignments[bot_id]][2],120.0)
                    self.robot_2_goals = astar_time(self.obstacle_grid_2 , self.robots[bot_id][:2] , [tx,ty],self.reserved_vertices,self.reserved_vertices,start_time_step=self.time_step)
                    


                    

                    if(self.robot_2_goals is not None):
                        #self.robot_2_goals = remove_linear_points(self.robot_2_goals)
                        #plot_the_path([bot_id] , [self.robot_2_goals])
                        
                        reserve_path(self.robot_2_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                        self.robot_2_goals[-1] = (tx,ty,self.robot_2_goals[-1][2])
                        print(f"Bot : {bot_id} \n Goals  {self.robot_2_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                        self.robot_2_planned = True
                else:
                    self.state_idx_2 += 1
                    self.robot_2_planned = False
        
        elif(self.states_order[self.state_idx_2] == 'MOVING'):
            if(self.controller_2 is None and self.robot_2_goals is not None):
                if(self.state_idx_2 == 1):
                    self.controller_2 = HolonomicPIDController(bot_id , 20 , False , self.assignments[2])
                elif(self.state_idx_2 == 4):
                    self.controller_2 = HolonomicPIDController(bot_id , 20)
                else:
                    self.controller_2 = HolonomicPIDController(bot_id)
                self.executor.add_node(self.controller_2)
                world_goals = []
                if(len(self.robot_2_goals) >= 2):
                    self.robot_2_goals.pop(0)
                for goal in self.robot_2_goals:
                    world_goals.append((goal[0]*40 + 20 , goal[1]*40 + 20 , goal[0]))
                if(self.state_idx_2 == 1):
                    world_goals[-1] = (world_goals[-1][0] - 20.0)/40.0,(world_goals[-1][1] - 20.0)/40.0,world_goals[-1][2]
                self.controller_2.set_goals(world_goals)


            if((self.controller_2 is None and self.controller_2_goals_reached is True )):
                self.executor.remove_node(self.controller_2)
                self.controller_2.destroy_node()
                self.controller_2 = None
                self.controller_2_goals_reached = False
                self.state_idx_2 += 1
            elif(self.controller_2 is not None and self.controller_2.all_goals_reached is True):
                self.executor.remove_node(self.controller_2)
                self.controller_2.destroy_node()
                self.controller_2 = None
                self.controller_2_goals_reached = False
                self.state_idx_2 += 1

            
        elif(self.states_order[self.state_idx_2] == 'DROP_PLANNING'):
            if(self.assignments[bot_id]%3 == 0):
                drop_zone = self.D1
            elif(self.assignments[bot_id]%3 == 1):
                drop_zone = [self.D2[0]+60.0,self.D2[1]]
            else:
                drop_zone = self.D3

            if(self.robot_2_planned is False):
                self.obstacle_grid_2 = np.zeros(shape = (61,61))
                self.update_grid(self.obstacle_grid_2 , bot_id)
                #self.robot_2_goals = self.astar(self.obstacle_grid_2 , self.robots[bot_id][:2] , drop_zone)
                if(self.controller_0 is not None and self.controller_0.return_time_step() != -1):
                    self.time_step = self.controller_0.return_time_step()
                elif(self.controller_4 is not None and self.controller_4.return_time_step() != -1):
                    self.time_step = self.controller_4.return_time_step()
                else:
                    self.time_step = 0
                print(f"{bot_id} : {self.time_step}")
                self.robot_2_goals = astar_time(self.obstacle_grid_2 , self.robots[bot_id][:2] , drop_zone ,self.reserved_vertices,self.reserved_vertices,start_time_step=self.time_step)
                
                if(self.robot_2_goals is not None):
                    #self.robot_2_goals = remove_linear_points(self.robot_2_goals)
                    #plot_the_path([bot_id] , [self.robot_2_goals])
                    reserve_path(self.robot_2_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                    print(f"Bot : {bot_id} \n Goals  {self.robot_2_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                
                    self.robot_2_planned = True

            else:
                self.state_idx_2 += 1
                self.robot_2_planned = False


        elif(self.states_order[self.state_idx_2] == 'DOCK_PLANNING'):
            if(self.robot_2_planned is False):
                self.obstacle_grid_2 = np.zeros(shape = (61,61))
                self.update_grid(self.obstacle_grid_2 , bot_id)
                #self.robot_2_goals = self.astar(self.obstacle_grid_2 , self.robots[bot_id][:2] , self.dock_2)
                if(self.controller_0 is not None and self.controller_0.return_time_step() != -1):
                    self.time_step = self.controller_0.return_time_step()
                elif(self.controller_4 is not None and self.controller_4.return_time_step() != -1):
                    self.time_step = self.controller_4.return_time_step()
                else:
                    self.time_step = 0
                print(f"{bot_id} : {self.time_step}")
                self.robot_2_goals = astar_time(self.obstacle_grid_2 , self.robots[bot_id][:2] , self.dock_2 ,self.reserved_vertices,self.reserved_vertices,start_time_step=self.time_step)
                
                if(self.robot_2_goals is not None):
                    #self.robot_2_goals = remove_linear_points(self.robot_2_goals)
                    #plot_the_path([bot_id] , [self.robot_2_goals])
                    reserve_path(self.robot_2_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                    print(f"Bot : {bot_id} \n Goals  {self.robot_2_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                    
                    self.robot_2_planned = True

            else:
                self.state_idx_2 += 1
                self.robot_2_planned = False

        elif(self.states_order[self.state_idx_2] == 'PICKUP'):
            if(self.time_2_1 is None):
                self.time_2_1 = self.get_clock().now().nanoseconds
                self.arm_control(bot_id , 90 , 90)

            elif(self.get_clock().now().nanoseconds - self.time_2_1 > 0.5e9 and self.time_2_2 is None):
                self.time_2_2 = self.get_clock().now().nanoseconds
                #self.send_attach_request(bot_id = 2)
            
            elif(self.time_2_2 is not None and self.get_clock().now().nanoseconds - self.time_2_2 > 0.5e9):
                self.arm_control(bot_id , 90 , 90)
                self.state_idx_2 += 1
                self.time_2_1 = None
                self.time_2_2 = None

        elif(self.states_order[self.state_idx_2] == 'DROP'):
            if(self.time_2_1 is None):
                self.time_2_1 = self.get_clock().now().nanoseconds
                self.arm_control(bot_id , 90 , 90)

            elif(self.get_clock().now().nanoseconds - self.time_2_1 > 0.5e9 and self.time_2_2 is None):
                self.time_2_2 = self.get_clock().now().nanoseconds
                #self.send_detach_request(bot_id = 2)
            
            elif(self.time_2_2 is not None and self.get_clock().now().nanoseconds - self.time_2_2 > 0.5e9):
                self.arm_control(bot_id , 90 , 90)
                self.state_idx_2 += 1
                self.time_2_1 = None
                self.time_2_2 = None
        
        elif(self.states_order[self.state_idx_2] == 'LOOK_FOR_NEXT'):
            if(self.task_lock is False):
                self.task_lock = True
                if len(self.remaining_tasks) != 0:
                    task_assigned = None
                    min_dist = None
                    for task in self.remaining_tasks:
                        dist = self.cost_function(self.robots[bot_id][:2] ,self.tasks[task][:2])
                        if min_dist is None or dist < min_dist:
                            min_dist = dist
                            task_assigned = task
                    self.assignments[bot_id] = task_assigned
                    self.state_idx_2 = 0
                    self.remaining_tasks.remove(task_assigned)
                else:
                    self.state_idx_2 += 1

                self.task_lock = False

    def update_4(self , bot_id = 4):
        if(self.state_idx_4 >= len(self.states_order)):
            self.get_logger().info("Mission Accomplished !")
            self.arm_control(bot_id=4)
            return
        elif(self.states_order[self.state_idx_4] == 'IDLE'):
            if(len(self.robots) == self.num_robots and len(self.tasks) == self.num_crates):
                if(self.task_allocated is False):
                    self.assignments = self.greedy_allocation(self.robots , self.tasks)
                    print(self.assignments)
                    self.task_allocated = True
                elif(self.robot_4_planned is False):    
                    self.obstacle_grid_4  = np.zeros(shape=(61,61))
                    self.update_grid(self.obstacle_grid_4 , bot_id)
                    #self.robot_4_goals = self.astar(self.obstacle_grid_4 , self.robots[bot_id][:2] , self.tasks[self.assignments[bot_id]][:2])
                    if(self.controller_0 is not None and self.controller_0.return_time_step() != -1):
                        self.time_step = self.controller_0.return_time_step()
                    elif(self.controller_2 is not None and self.controller_2.return_time_step() != -1):
                        self.time_step = self.controller_2.return_time_step()
                    else:
                        self.time_step = 0
                    print(f"{bot_id} : {self.time_step}")
                    self.robot_4_goals = astar_time(self.obstacle_grid_4 , self.robots[bot_id][:2] , self.tasks[self.assignments[bot_id]] ,self.reserved_vertices,self.reserved_vertices,start_time_step=self.time_step)
                    if(self.robot_4_goals is not None):
                        #self.robot_4_goals = remove_linear_points(self.robot_4_goals)
                        #plot_the_path([bot_id] , [self.robot_4_goals])
                        reserve_path(self.robot_4_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                        print(f"Bot : {bot_id} \n Goals  {self.robot_4_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                        self.robot_4_planned = True
                else:
                    self.state_idx_4 += 1
                    self.robot_4_planned = False
        
        elif(self.states_order[self.state_idx_4] == 'MOVING'):
            if(self.controller_4 is None and self.robot_4_goals is not None):
                if(self.state_idx_4 == 1):
                    self.controller_4 = HolonomicPIDController(bot_id , 145 , False , self.assignments[4])
                elif(self.state_idx_4 == 4):
                    self.controller_4 = HolonomicPIDController(bot_id , 20)
                else:
                    self.controller_4 = HolonomicPIDController(bot_id)
                self.executor.add_node(self.controller_4)
                if(len(self.robot_4_goals) >= 2):
                    self.robot_4_goals.pop(0)
                world_goals = []
                for goal in self.robot_4_goals:
                    world_goals.append((goal[0]*40 + 20 , goal[1]*40 + 20 , goal[2]))
                self.controller_4.set_goals(world_goals)


            if((self.controller_4 is None and self.controller_4_goals_reached is True )):
                self.executor.remove_node(self.controller_4)
                self.controller_4.destroy_node()
                self.controller_4 = None
                self.controller_4_goals_reached = False
                self.state_idx_4 += 1
            elif(self.controller_4 is not None and self.controller_4.all_goals_reached is True):
                self.executor.remove_node(self.controller_4)
                self.controller_4.destroy_node()
                self.controller_4 = None
                self.controller_4_goals_reached = False
                self.state_idx_4 += 1

            
        elif(self.states_order[self.state_idx_4] == 'DROP_PLANNING'):
            if(self.assignments[bot_id]%3 == 0):
                drop_zone = [self.D1[0]-80.0,self.D1[1]]
            elif(self.assignments[bot_id]%3 == 1):
                drop_zone = [self.D2[0]-60.0,self.D2[1]]
            else:
                drop_zone = self.D3

            if(self.robot_4_planned is False):
                self.obstacle_grid_4 = np.zeros(shape = (61,61))
                self.update_grid(self.obstacle_grid_4 , bot_id)
                #self.robot_4_goals = self.astar(self.obstacle_grid_4 , self.robots[bot_id][:2] , drop_zone)
                if(self.controller_0 is not None and self.controller_0.return_time_step() != -1):
                    self.time_step = self.controller_0.return_time_step()
                elif(self.controller_2 is not None and self.controller_2.return_time_step() != -1):
                    self.time_step = self.controller_2.return_time_step()
                else:
                    self.time_step = 0
                print(f"{bot_id} : {self.time_step}")
                self.robot_4_goals = astar_time(self.obstacle_grid_4 , self.robots[bot_id][:2] , drop_zone ,self.reserved_vertices,self.reserved_vertices,start_time_step=self.time_step)

                if(self.robot_4_goals is not None):
                    #self.robot_4_goals = remove_linear_points(self.robot_4_goals)
                    #plot_the_path([bot_id] , [self.robot_4_goals])
                    reserve_path(self.robot_4_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                    print(f"Bot : {bot_id} \n Goals  {self.robot_4_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                    
                    self.robot_4_planned = True

            else:
                self.state_idx_4 += 1
                self.robot_4_planned = False


        elif(self.states_order[self.state_idx_4] == 'DOCK_PLANNING'):
            if(self.robot_4_planned is False):
                self.obstacle_grid_4 = np.zeros(shape = (61,61))
                self.update_grid(self.obstacle_grid_4 , bot_id)
                #self.robot_4_goals = self.astar(self.obstacle_grid_4 , self.robots[bot_id][:2] , self.dock_4)
                if(self.controller_0 is not None and self.controller_0.return_time_step() != -1):
                    self.time_step = self.controller_0.return_time_step()
                elif(self.controller_2 is not None and self.controller_2.return_time_step() != -1):
                    self.time_step = self.controller_2.return_time_step()
                else:
                    self.time_step = 0
                print(f"{bot_id} : {self.time_step}")
                self.robot_4_goals = astar_time(self.obstacle_grid_4 , self.robots[bot_id][:2] , self.dock_4 ,self.reserved_vertices,self.reserved_vertices,max_time=50,start_time_step=self.time_step)
                if(self.robot_4_goals is not None):
                    #self.robot_4_goals = remove_linear_points(self.robot_4_goals)
                    #plot_the_path([bot_id] , [self.robot_4_goals])
                    reserve_path(self.robot_4_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                    print(f"Bot : {bot_id} \n Goals  {self.robot_4_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                    
                    self.robot_4_planned = True

            else:
                self.state_idx_4 += 1
                self.robot_4_planned = False

        elif(self.states_order[self.state_idx_4] == 'PICKUP'):
            if(self.time_4_1 is None):
                self.time_4_1 = self.get_clock().now().nanoseconds
                self.arm_control(bot_id , 15 , 90)

            elif(self.get_clock().now().nanoseconds - self.time_4_1 > 2e9 and self.time_4_2 is None):
                self.time_4_2 = self.get_clock().now().nanoseconds
                self.send_attach_request(bot_id = 4)
            
            elif(self.time_4_2 is not None and self.get_clock().now().nanoseconds - self.time_4_2 > 2e9):
                self.arm_control(bot_id , 30 , 90)
                self.state_idx_4 += 1
                self.time_4_1 = None
                self.time_4_2 = None

        elif(self.states_order[self.state_idx_4] == 'DROP'):
            if(self.time_4_1 is None):
                self.time_4_1 = self.get_clock().now().nanoseconds
                self.arm_control(bot_id , 20 , 90)

            elif(self.get_clock().now().nanoseconds - self.time_4_1 > 2e9 and self.time_4_2 is None):
                self.time_4_2 = self.get_clock().now().nanoseconds
                self.send_detach_request(bot_id = 4)
            
            elif(self.time_4_2 is not None and self.get_clock().now().nanoseconds - self.time_4_2 > 2e9):
                self.arm_control(bot_id , 130 , 90)
                self.state_idx_4 += 1
                self.time_4_1 = None
                self.time_4_2 = None   

        elif(self.states_order[self.state_idx_4] == 'LOOK_FOR_NEXT'):
            if(self.task_lock is False):
                self.task_lock = True
                if len(self.remaining_tasks) != 0:
                    task_assigned = None
                    min_dist = None
                    for task in self.remaining_tasks:
                        dist = self.cost_function(self.robots[bot_id][:2] ,self.tasks[task][:2])
                        if min_dist is None or dist < min_dist:
                            min_dist = dist
                            task_assigned = task
                    self.assignments[bot_id] = task_assigned
                    self.state_idx_4 = 0
                    self.remaining_tasks.remove(task_assigned)
                else:
                    self.state_idx_4 += 1

                self.task_lock = False    

            

            

def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = fsm(exec = executor )
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

            


            




            


        
            

            

            


