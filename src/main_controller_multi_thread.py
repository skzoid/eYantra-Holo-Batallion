#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from std_msgs.msg import Bool
from hb_interfaces.msg import Pose2D, Poses2D,BotCmd ,BotCmdArray
from std_srvs.srv import SetBool
from rclpy.executors import SingleThreadedExecutor,MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import heapq
import threading

goals_reached_lock_0 = threading.Lock()
goals_reached_lock_2 = threading.Lock()
goals_reached_lock_4 = threading.Lock()




class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
#-----------------------------PID Compute Steps--------------------------------------------------------------
        # 1. Accumulate the error over time for the Integral term
        self.integral += error * dt

        # 2. Compute the change in error for the Derivative term
        derivative =  (error - self.prev_error)/dt

        # 3. Calculate the PID output:
        output = self.kp*error + self.ki*self.integral + self.kd*derivative

        # 4. Store the current error for use in the next iteration
        self.prev_error = error

        # 5. Limit (clip) the output between [-max_out, +max_out] to avoid unsafe velocities
        #print(output)
        return np.clip(output, -self.max_out, self.max_out)

#------------------------------------------------------------------------------------------------------------

    
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
class HolonomicPIDController(Node):
    def __init__(self , bot_id , final_threshold = 10.0 , dock = False , drop = False , assignment = None , drop_angle = np.pi/2):
        super().__init__('holonomic_pid_controller')  # initializing ros node

        # ---------------- Robot Parameters ----------------
        self.ROBOT_RADIUS = 66  # L: Distance from center to wheel (in mm)
        self.WHEEL_RADIUS = 19.0  # r: Radius of the wheels (in mm)
        self.max_vel = 2000.0       # Maximum linear velocity for the robot body (mm/s)
        self.MAX_WHEEL_VEL = 20.0  # Maximum angular velocity for a wheel (rad/s)
        
        # 1. Robot ID(s)
        self.robot_id = bot_id

        # 2. Current pose of the robot:
        #    - Updated from the /bot_pose topic in the callback function.
        #    - Stores [x, y, θ] information for the active robot.
        self.current_pose = None
        self.crate_pose = None
        self.start_time = None
        self.time_set = False
        self.assignment = assignment

        # 3. Goal tracking index
        self.goal_index = 0
        self.flag = 0

        # 4. Timing information:
        #    - Used to calculate the time difference (dt) between control loop iterations.
        self.last_time = self.get_clock().now().nanoseconds/1e9

        # 5. Threshold for goal completion:
        #    - Defines the acceptable error tolerance for x, y, and θ.
        #    - Example: if error < 5 units → goal considered reached.
        self.goal_threshold = {'pos': 15.0, 'theta': 0.04}
        self.final_threshold = final_threshold
        self.attached = False
        self.detached = False
        self.goals = []
        self.target_angle = None
        self.target_flag = False
        self.drop = drop
        self.dock = dock
        
        self.drop_angle = drop_angle


        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 10.0, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel},
            'y': {'kp': 10.0, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel},
            'theta': {'kp': 0.3, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel}
        }

        # Initialize PIDs
        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        self.file = open("data.txt","a")

        # ---------------- ROS 2 Publishers & Subscribers ----------------
        
        # Write a subscriber for /bot_pose
        self.pose_sub = self.create_subscription(
            Poses2D,
            '/bot_pose',
            self.pose_cb,
            10 , callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.publisher = self.create_publisher(
            BotCmdArray, '/bot_cmd', 100 , callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        self.crate_pose_sub = self.create_subscription(
            Poses2D,
            'crate_pose',
            self.crate_pose_cb,
            10 , callback_group=MutuallyExclusiveCallbackGroup()
        )
        if(bot_id == 0):
            self.sensor_subscription = self.create_subscription(Bool , '/sensor0' ,self.sensor_callback , 10 , callback_group=MutuallyExclusiveCallbackGroup())
        elif(bot_id == 2):
            self.sensor_subscription = self.create_subscription(Bool , '/sensor2' ,self.sensor_callback , 10 , callback_group=MutuallyExclusiveCallbackGroup())
        elif(bot_id == 4):
            self.sensor_subscription = self.create_subscription(Bool , '/sensor4' ,self.sensor_callback , 10 , callback_group=MutuallyExclusiveCallbackGroup())

        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.04, self.control_cb , callback_group=MutuallyExclusiveCallbackGroup())   # ~30ms = 33 Hz
        self.wait_start = self.get_clock().now()
        self.waiting = False

        self.all_goals_reached = False
        self.halt = False
        self.bot_id = bot_id
        self.close_flag = False
        self.detection = False
        self.p_ref = None
        self.t_ref = None
        self.target_vel = [0.0,0.0]
        self.del_t = 0.5

        self.bot_pose_lock = threading.Lock()
        self.crate_pose_lock = threading.Lock()
        self.detection_lock = threading.Lock()
        

    def set_goals(self , goals):

            self.goals = goals
    # ---------------- Subscriber Callback ----------------
    def pose_cb(self, msg):
        """
        Callback function for /bot_pose topic.
        This function is executed each time a message is received.

        Steps:
        1. Iterate through all poses in the incoming message.
        2.  Update self.current_pose with this robot’s pose.
        """
        for pose in msg.poses:
            if pose.id == self.robot_id:
                yaw_in_degrees = pose.w
                yaw_in_radians = math.radians(yaw_in_degrees)
                with self.bot_pose_lock:
                    self.current_pose = [pose.x, pose.y, yaw_in_radians]

                # for debugging
                #self.get_logger().info(f'Bot Pose Updated: x={self.current_pose[0]:.2f}, y={self.current_pose[1]:.2f}, yaw_rad={self.current_pose[2]:.2f}')
                return

    # ---------------- Control Loop ----------------
    def crate_pose_cb(self , msg : Poses2D):
        for pose in msg.poses:
            if pose.id == self.assignment:
                yaw_in_degrees = pose.w
                yaw_in_radians = math.radians(yaw_in_degrees)
                with self.crate_pose_lock:
                    self.crate_pose = [pose.x, pose.y, yaw_in_radians]



    def sensor_callback(self , msg):
        with self.detection_lock:
            self.detection = msg.data



    
    def set_target_angle(self , crate_pos , bot_pos):
        dx = crate_pos[0] - bot_pos[0]
        dy = crate_pos[1] - bot_pos[1]

        theta = None

        
        if(dx > 0 and dy > 0):
            theta = math.atan(dy/dx)
        elif(dx < 0 and dy > 0):
            theta = np.pi - math.atan(-dy/dx)
        elif(dx < 0 and dy < 0):
            theta = np.pi + math.atan(dy/dx)
        else:
            theta = 2 * np.pi - math.atan(-dy/dx)
        
        return theta


    def dist(self , a , b):
        return np.sqrt((a[0] - b[0])**2 + (a[1]-b[1])**2)
    

    
    def control_cb(self):

        """
        Control loop callback executed periodically by the ROS 2 timer.

        Main Steps:
        1. Check if the current pose is available; if not, exit.
        2. Compute the time difference (dt) since the last control cycle.
        3. Get the current robot pose (x, y, θ).
        4. If all goals are completed → stop the robot.
        5. Select the current goal (x, y, θ) from the goals list.
        6. Compute errors in x, y, and θ between current pose and goal.
        7. Use PID controllers to calculate required body velocities [vx, vy, ω].
        8. Convert body velocities into individual wheel velocities.
        9. Limit (clip) wheel velocities within safe bounds.
        10. Publish the wheel velocities to the motor controller.
        11. Check if the goal is reached:
              - If yes → update goal index, reset PIDs, and move to the next goal.
        """


        if(self.time_set is False):
            self.time_set = True
            self.start_time = self.get_clock().now().nanoseconds/1e9
            print("-"*15 , f"Start time : {self.start_time}" , "-"*15)
        
        if(self.goal_index >= len(self.goals)):
            self.goal_index -= 1


        if self.current_pose is None :
            self.get_logger().info(f'Waiting for pose data...{self.robot_id}')
            return

        if len(self.goals) == 0:
            self.get_logger().info(f'Waiting for goals...{self.robot_id}')
            return
        else:

            self.final_goal = list(self.goals[-1])


        if(self.p_ref is None):
            self.p_ref = self.current_pose[:2]
            self.t_ref = self.get_clock().now().nanoseconds/1e9
            #self.del_t = self.goals[1][2] - self.goals[0][2]
            print(len(self.goals) , self.bot_id)
            self.target_vel[0] = (self.goals[1][0] - self.p_ref[0])/self.del_t
            self.target_vel[1] = (self.goals[1][1] - self.p_ref[1])/self.del_t


        

    
        # Time delta
        now = self.get_clock().now().nanoseconds/1e9
        dt = now - self.last_time
        if dt <= 0:
            return
        self.last_time = now

        # Current robot pose
        with self.bot_pose_lock:
            x, y, theta = self.current_pose



        if(self.goal_index < len(self.goals) - 1):
            target_x, target_y = self.p_ref[0],self.p_ref[1]
            target_x += self.target_vel[0]*(now - self.t_ref)
            target_y += self.target_vel[1]*(now - self.t_ref)
        else:
            target_x = self.final_goal[0]
            target_y = self.final_goal[1]


            


        with self.crate_pose_lock:
            if((self.drop is False) and (self.dock is False)):
                if self.target_flag is False:
                    target_theta = self.set_target_angle([target_x , target_y] , [x,y]) 
                else:
                    print(self.bot_id,self.assignment , self.crate_pose)
                    target_theta = self.set_target_angle(self.crate_pose[:2] , [x,y])
            else:
                target_theta = self.drop_angle


        # Errors
        error_x = target_x - x
        error_y = target_y - y

        error_theta = target_theta - theta 
        if self.bot_id == 4:
            error_theta -= 0.05
        error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
        
        

        # PID outputs
        # GLOBAL body velocities

        vx_global = self.pid_x.compute(-error_x, dt)
        vy_global = self.pid_y.compute(-error_y, dt)
        omega = self.pid_theta.compute(error_theta, dt)

        #print(vx_global , vy_global , omega)



        # Converting Global velocities to LOCAL robot frame velocities
        vx_local = vx_global * math.sin(theta) - vy_global * math.cos(theta)
        vy_local = vx_global * math.cos(theta) + vy_global * math.sin(theta)

        # Convert to wheel velocities (custom equations)

        # Angles of the wheels relative to the robot's X-axis
        # Wheel 1: 30 deg, Wheel 2: 150 deg, Wheel 3: 270 deg
        theta1 = math.radians(30) 
        theta2 = math.radians(150) 
        theta3 = math.radians(270) 

        pos_error = math.sqrt(error_x**2 + error_y**2)
        # if(self.bot_id==0):
        #     now = self.get_clock().now().nanoseconds/1e9
        #     print(f"pos_error : {pos_error}")
        #     self.file.write(f"{pos_error},{now}\n")
        #     self.file.flush()



        w1 = (1/self.WHEEL_RADIUS) * (math.sin(theta1)*vx_local - math.cos(theta1)*vy_local + (self.ROBOT_RADIUS)*omega)
        w2 = (1/self.WHEEL_RADIUS) * (math.sin(theta2)*vx_local - math.cos(theta2)*vy_local + (self.ROBOT_RADIUS)*omega)
        w3 = (1/self.WHEEL_RADIUS) * (math.sin(theta3)*vx_local - math.cos(theta3)*vy_local + (self.ROBOT_RADIUS)*omega)
        wheel_vel = [w1, w2, w3]



        # Goal check

        #if(pos_error < self.goal_threshold['pos'] and abs(error_theta) < self.goal_threshold['theta'] ):
        dist = self.dist(self.final_goal , [x,y])
        # if(self.bot_id == 2):
        #     print(f"Final Goal : {self.final_goal} --- Current : {x},{y} --- Distance : {dist}")
        if(dist < self.final_threshold):
            self.pid_x.kp = 0.0
            self.pid_x.kd = 0.0
            self.pid_y.kp = 0.0
            self.pid_y.kd = 0.0
            self.pid_theta.kp = 1.0

            self.target_flag = True
            print(f"{self.bot_id} -- Stopped!")
            
            with self.detection_lock:
                if((  abs(error_theta) < 0.1 and self.detection and (not self.drop) and (not self.dock) ) or (self.drop and abs(error_theta) < 0.05) or (self.dock) ):

                    
                    wheel_vel = [0.0 , 0.0 , 0.0]
                    if self.bot_id == 0:
                        with goals_reached_lock_0:
                            self.all_goals_reached = True
                        print(f"{self.bot_id} -- Goal_Reached -- Detection :{self.detection} -- Num_goals : {self.goal_index+1}")
                        now = self.get_clock().now().nanoseconds/1e9
                        print("-"*15 , f"End Time : {now} , Time Taken : {now - self.start_time}" , "-"*15)
                    if self.bot_id == 2:
                        with goals_reached_lock_2:
                            self.all_goals_reached = True
                        print(f"{self.bot_id} -- Goal_Reached -- Detection :{self.detection} -- Num_goals : {self.goal_index+1}")
                        now = self.get_clock().now().nanoseconds/1e9
                        print("-"*15 , f"End Time : {now} , Time Taken : {now - self.start_time}" , "-"*15)
                    if self.bot_id == 4:
                        with goals_reached_lock_4:
                            self.all_goals_reached = True
                        print(f"{self.bot_id} -- Goal_Reached -- Detection :{self.detection} -- Num_goals : {self.goal_index+1}")
                        now = self.get_clock().now().nanoseconds/1e9
                        print("-"*15 , f"End Time : {now} , Time Taken : {now - self.start_time}" , "-"*15)

        elif(self.pid_x.kp == 0):
            self.pid_x.kp = 0.25
            self.pid_y.kp = 0.25
            self.pid_theta.kp = 0.0

        elif(now - self.t_ref >= self.del_t):

            self.goal_index += 1
            #

            
            if self.goal_index < len(self.goals) - 1:
                #self.del_t =self.goals[self.goal_index+1][2] - self.goals[self.goal_index][2]
                self.p_ref = list(self.goals[self.goal_index])[:2]
                self.t_ref = self.get_clock().now().nanoseconds/1e9
                self.target_vel[0] = (self.goals[self.goal_index+1][0] - self.goals[self.goal_index][0])/self.del_t
                self.target_vel[1] = (self.goals[self.goal_index+1][1] - self.goals[self.goal_index][1])/self.del_t
            else:
                pass
            #print(self.bot_id , f": Goal : {self.goal_index}")

        
        self.publish_wheel_velocities(wheel_vel)




    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel , angle = [130.0 , 90.0]):
        # Wheel velocity array (Float64MultiArray)
        # Order: [Left wheel speed, Right wheel speed, Rear wheel speed]
        msg = BotCmd()

        for i in range(len(wheel_vel)):
            if (0.05 < wheel_vel[i]):
                    wheel_vel[i] += 9.0
            if(-0.05 > wheel_vel[i]):
                wheel_vel[i] -= 3.0  

        msg.m1 = np.float64((wheel_vel[0]+90))
        msg.m2 = np.float64(wheel_vel[1] + 90)
        msg.m3 = np.float64(wheel_vel[2]+90)
        msg.id = self.bot_id
        if self.bot_id == 2:
            msg.elbow = np.float64(angle[1]-10)
        else:
            msg.elbow = np.float64(angle[1])
        if self.bot_id == 0:
            msg.base = np.float64(angle[0])
        else:
            msg.base = np.float64(angle[0] + 10)
        Msg = BotCmdArray()
        Msg.cmds = [msg]
        self.publisher.publish(Msg)
        # if(self.bot_id == 0):
        #     print(Msg)

class fsm(Node):
    def __init__(self , exec):
        super().__init__("fsm_node")

        self.executor = exec

        self.num_crates = 6
        self.num_robots = 3

        self.robots = {}
        self.tasks = {}

        self.assignments = {}
        self.remaining_tasks = set()
        self.task_lock = False
               
        self.pose_sub = self.create_subscription(Poses2D , '/bot_pose' , self.pose_cb , 10 , callback_group=MutuallyExclusiveCallbackGroup())
        self.crate_sub = self.create_subscription(Poses2D , '/crate_pose' , self.crate_cb , 10 , callback_group=MutuallyExclusiveCallbackGroup())
        self.publisher = self.create_publisher(BotCmdArray, '/bot_cmd', 100 , callback_group=MutuallyExclusiveCallbackGroup())
    

        self.attach_client_0 = self.create_client(SetBool , '/attach/bot0', callback_group=MutuallyExclusiveCallbackGroup())
        self.attach_client_2 = self.create_client(SetBool , '/attach/bot2', callback_group=MutuallyExclusiveCallbackGroup())
        self.attach_client_4 = self.create_client(SetBool , '/attach/bot4', callback_group=MutuallyExclusiveCallbackGroup())


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


        self.states_order = ['IDLE' , 'MOVING' , 'PICKUP' , 'DROP_PLANNING' , 'MOVING' , 'DROP', 'LOOK_FOR_NEXT' , 'DOCK_PLANNING' , 'MOVING' ]
     
        self.state_idx_0 = 0
        self.state_idx_2 = 0
        self.state_idx_4 = 0

        self.D1 = [1215 , 1215]
        self.D2 = [820 , 2017.5]
        self.D3 = [1616 , 2017.5]

        self.D1_bookings = {0:[[1215,1215],[1295,1215],[1155,1215],[1227,1215],[1203,1225]] , 1:[[1255,1215],[1224,1215],[1212,1215],[1224,1215]]}
        self.D2_bookings = {0:[[820,2017.5],[900,2017.5],[814,2017.5],[832,2017.5],[806,2017.5]] , 1:[[860,2037.5],[817,2017.5],[829,2017.5],[811,2017.5]]}
        self.D3_bookings = {0:[[1616,2017.5],[1686,2017.5],[1610,2017.5],[1628,2017.5],[1604,2017.5]] , 1:[[1651,2017.5],[1613,2017.5],[1625,2017.5],[1607,2017.5]]}
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
        self.timer_start = None

        self.lock = threading.Lock()
        self.planning_lock = threading.Lock()
        self.task_lock = threading.Lock()




        self.bots_in_radius = []

        for client in [self.attach_client_0, self.attach_client_2, self.attach_client_4]:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn("Attach service not available")

            


        # while(self.task_allocated is False):
        #     if not (len(self.robots) == self.num_robots and len(self.tasks) == self.num_crates):
        #         print(len(self.robots) , len(self.tasks))
        #         continue
        #     self.assignments = self.greedy_allocation(self.robots , self.tasks)
        #     self.allocate_drop_location()
        #     print(f"\n\n\n Drop_Locations : {self.drop_allocated}")
        #     self.task_allocated = True
        
        self.timer_0 = self.create_timer(0.1 , self.update_0 , callback_group= MutuallyExclusiveCallbackGroup())
        self.timer_2 = self.create_timer(0.1 , self.update_2 , callback_group=MutuallyExclusiveCallbackGroup())
        self.timer_4 = self.create_timer(0.1 , self.update_4, callback_group=MutuallyExclusiveCallbackGroup())
        self.timer = self.create_timer(0.01 , self.update_time, callback_group=MutuallyExclusiveCallbackGroup())
        #self.collision_checker_timer = self.create_timer(0.1 , self.collision_checker)
        #self.stop_and_go_timer = self.create_timer(0.1 , self.stop_and_go)

        self.reserved_edges = {}
        self.reserved_vertices = {}



                


    def update_time(self):
        if self.timer_start is not None:
            now = self.get_clock().now().nanoseconds/1e9
            if(now - self.timer_start > 0.5 * (self.time_step + 1)):
                self.time_step += 1
                



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
    
    def arm_control(self , bot_id ,base = 120, elbow = 90):
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
        def give_drop_boundries(c1,c2,c3,c4):
            pts = []

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
            
            l4 = []
            for m in ms:
                pt = c3 * m + c4 * (1-m)
                l4.append(pt)

            pts = l1 + l2 + l3 + l4
            return pts
        
        D1_1 = np.array([1020.0,1355.0])
        D1_2 = np.array([1410.0,1355.0])
        D1_3 = np.array([1410.0,1075.0])
        D1_4 = np.array([1020.0,1075.0])

        D2_1 = np.array([1466.0,2117.5])
        D2_2 = np.array([1766.0,2117.5])
        D2_3 = np.array([1766.0,1917.5])
        D2_4 = np.array([1466.0,1917.5])

        D3_1 = np.array([670.0,2117.5])
        D3_2 = np.array([970.0,2117.5])
        D3_3 = np.array([970.0,1917.5])
        D3_4 = np.array([670.0,1917.5])

        pts_1 = give_drop_boundries(D1_1,D1_2,D1_3,D1_4)
        pts_2 = give_drop_boundries(D2_1,D2_2,D2_3,D2_4)
        pts_3 = give_drop_boundries(D3_1,D3_2,D3_3,D3_4)

        pts = pts_1 + pts_2 + pts_3
        
        self.add_dock_obstacles(grid , pts , grid_size)


    def add_dock_obstacles(self , grid , pts , grid_size = 40):
        for pt in pts:
            x = int(pt[0]//grid_size)
            y = int(pt[1]//grid_size)

            grid[x][y] = 1




                
                        

    



    def pose_cb(self, msg):

        for pose in msg.poses:
                yaw_in_degrees = pose.w
                yaw_in_radians = math.radians(yaw_in_degrees)
                self.robots[pose.id] = [pose.x, pose.y, yaw_in_radians]
                

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
                        self.drop_allocated[crate_id] = [1,self.D1_bookings[1][int(self.D1_idx - self.num_D1/2)]]
                        self.D1_idx += 1
                if crate_id % 3 == 1:
                    if self.D2_idx <= self.num_D2/2:
                        self.drop_allocated[crate_id] = [0,self.D2_bookings[0][self.D2_idx]]
                        self.D2_idx += 1
                    else:
                        self.drop_allocated[crate_id] = [1,self.D2_bookings[1][int(self.D2_idx - self.num_D2/2)]]
                        self.D2_idx += 1
                if crate_id % 3 == 2:
                    if self.D3_idx <= self.num_D3/2:
                        self.drop_allocated[crate_id] = [0,self.D3_bookings[0][self.D3_idx]]
                        self.D3_idx += 1
                    else:
                        self.drop_allocated[crate_id] = [1,self.D3_bookings[1][int(self.D3_idx - self.num_D3/2)]]
                        self.D3_idx += 1
    
    def give_drop_location(self , bot_id , crate_id , threshold):
        crate_pose = self.drop_allocated[crate_id][1]
        p1 = [crate_pose[0],crate_pose[1]+threshold]
        p2 = [crate_pose[0],crate_pose[1]-threshold]
        
        d1 = self.cost_function(self.robots[bot_id][:2] , p1)
        d2 = self.cost_function(self.robots[bot_id][:2] , p2)

        if(d1 < d2):
            return p1,3*np.pi/2
        else:
            return p2,np.pi/2
        
        
    def crate_cb(self , msg):
            
        for pose in msg.poses:
            
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
    

   
    

    def update_0(self , bot_id = 0 , thres = 215):
        if(self.state_idx_0 >= len(self.states_order)):
            self.get_logger().info("Mission Accomplished !")
            self.arm_control(bot_id=bot_id)
            return
        elif(self.states_order[self.state_idx_0] == 'IDLE'):
            with self.task_lock:
                if self.task_allocated is False:
                    if not (len(self.robots) == self.num_robots and len(self.tasks) == self.num_crates):
                        print(len(self.robots) , len(self.tasks))
                        return
                    self.assignments = self.greedy_allocation(self.robots , self.tasks)
                    self.allocate_drop_location()
                    print(f"\n\n\n Drop_Locations : {self.drop_allocated}")
                    self.task_allocated = True

                

            if(self.robot_0_planned is False):    
                self.obstacle_grid_0  = np.ones(shape = (61,61))*5
                self.update_grid(self.obstacle_grid_0 , bot_id)

                print(f"{bot_id} : {self.time_step}")
                tx,ty = self.calculate_pickup_pose(self.tasks[self.assignments[bot_id]][0],self.tasks[self.assignments[bot_id]][1],self.tasks[self.assignments[bot_id]][2],thres)
                print(f"Bot_id : {bot_id} , crate_id : {self.assignments[bot_id]} , Pose : {self.tasks[self.assignments[bot_id]]}")

                with self.planning_lock:
                    self.robot_0_goals = astar_time(self.obstacle_grid_0 , self.robots[bot_id][:2] , [tx,ty] ,self.reserved_vertices,self.reserved_vertices,bot_id,start_time_step=self.time_step)
                    
                    if(self.robot_0_goals is not None):

                        reserve_path(self.robot_0_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                        self.robot_0_goals[-1] = (tx,ty,self.robot_0_goals[-1][2])
                        print(f"Bot : {bot_id} \n Goals  {self.robot_0_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                        self.robot_0_planned = True

            else:
                self.state_idx_0 += 1
                self.robot_0_planned = False

        
        elif(self.states_order[self.state_idx_0] == 'MOVING'):

            if(self.controller_0 is None and self.robot_0_goals is not None):
                self.arm_control(bot_id)
                if(self.timer_start is None):
                    with self.lock :
                        self.timer_start = self.get_clock().now().nanoseconds/1e9

                if(self.state_idx_0 == 1):
                    self.controller_0 = HolonomicPIDController(bot_id , 10 ,assignment = self.assignments[0])

                elif(self.state_idx_0 == 4):
                    drop_zone,yaw = self.give_drop_location(bot_id , self.assignments[bot_id] , thres )
                    self.controller_0 = HolonomicPIDController(bot_id , 10 , drop_angle=yaw , drop = True)

                else:
                    self.controller_0 = HolonomicPIDController(bot_id ,dock = True)

                
                world_goals = []
                if(len(self.robot_0_goals) >= 2):
                    self.robot_0_goals.pop(0)
                for goal in self.robot_0_goals:
                    world_goals.append((goal[0]*40 + 20 , goal[1]*40 + 20 , goal[2]))
                if(self.state_idx_0 == 1 or self.state_idx_0 == 4):
                    world_goals[-1] = (world_goals[-1][0] - 20.0)/40.0,(world_goals[-1][1] - 20.0)/40.0,world_goals[-1][2]
                self.controller_0.set_goals(world_goals)

                with self.lock:
                    self.executor.add_node(self.controller_0)


            elif(self.controller_0 is not None ):
                with goals_reached_lock_0:
                    if self.controller_0.all_goals_reached is False:
                        return
                with self.lock:
                    self.executor.remove_node(self.controller_0)
                #self.controller_0.destroy_node()
                self.controller_0 = None
                self.state_idx_0 += 1



            
        elif(self.states_order[self.state_idx_0] == 'DROP_PLANNING'):


            if self.drop_allocated[self.assignments[bot_id]][0] == 0:
                drop_zone,yaw = self.give_drop_location(bot_id , self.assignments[bot_id] , thres)
            else:
                drop_zone,yaw = self.give_drop_location(bot_id , self.assignments[bot_id] , thres - 15.0)

            if(self.robot_0_planned is False):
                self.obstacle_grid_0 = np.ones(shape = (61,61))*5
                self.update_grid(self.obstacle_grid_0 , bot_id)
                print(f"{bot_id} : {self.time_step}")
                with self.planning_lock:
                    self.robot_0_goals = astar_time(self.obstacle_grid_0 , self.robots[bot_id][:2] , drop_zone ,self.reserved_vertices,self.reserved_vertices,bot_id,start_time_step=self.time_step)
                    if self.robot_0_goals is not None:
                        
                        reserve_path(self.robot_0_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                        self.robot_0_goals[-1] = (drop_zone[0],drop_zone[1],self.robot_0_goals[-1][2])
                        print(f"Bot : {bot_id} \n Goals  {self.robot_0_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices}")
                        self.robot_0_planned = True

            else:
                self.state_idx_0 += 1
                self.robot_0_planned = False


        elif(self.states_order[self.state_idx_0] == 'DOCK_PLANNING'):
            if(self.robot_0_planned is False):
                self.obstacle_grid_0 = np.ones(shape = (61,61))*5
                self.update_grid(self.obstacle_grid_0 , bot_id)

                print(f"{bot_id} : {self.time_step}")
                with self.planning_lock:
                    self.robot_0_goals = astar_time(self.obstacle_grid_0 , self.robots[bot_id][:2] , self.dock_0 ,self.reserved_vertices,self.reserved_vertices,bot_id,start_time_step=self.time_step)
                    if self.robot_0_goals is not None:
                        reserve_path(self.robot_0_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                        print(f"Bot : {bot_id} \n Goals  {self.robot_0_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")


                        self.robot_0_planned = True

            else:
                self.state_idx_0 += 1
                self.robot_0_planned = False

        elif(self.states_order[self.state_idx_0] == 'PICKUP'):
            if(self.time_0_1 is None):
                self.time_0_1 = self.get_clock().now().nanoseconds
                self.arm_control(bot_id , 50 , 10)
                self.arm_control(bot_id , 45 , 10)



            elif(self.get_clock().now().nanoseconds - self.time_0_1 > 2e9 and self.time_0_2 is None):
                self.time_0_2 = self.get_clock().now().nanoseconds
                self.send_attach_request(bot_id = 0)
            
            elif(self.time_0_2 is not None and self.get_clock().now().nanoseconds - self.time_0_2 > 2e9):
                self.arm_control(bot_id , 90 , 10)
                self.state_idx_0 += 1
                self.time_0_1 = None
                self.time_0_2 = None

        elif(self.states_order[self.state_idx_0] == 'DROP'):
            if(self.time_0_1 is None):
                self.time_0_1 = self.get_clock().now().nanoseconds
                if self.drop_allocated[self.assignments[bot_id]][0] == 0:
                    self.arm_control(bot_id , 120,5)
                    self.arm_control(bot_id , 45 , 10)
                elif self.drop_allocated[self.assignments[bot_id]][0] == 1:
                    self.arm_control(bot_id , 80 , 35)
                    

            elif(self.get_clock().now().nanoseconds - self.time_0_1 > 2e9 and self.time_0_2 is None):
                self.time_0_2 = self.get_clock().now().nanoseconds
                self.send_detach_request(bot_id = 0)
            
            elif(self.time_0_2 is not None and self.get_clock().now().nanoseconds - self.time_0_2 > 2e9):
                if self.drop_allocated[self.assignments[bot_id]][0] == 0:
        
                    self.arm_control(bot_id , 90 , 10)
                elif self.drop_allocated[self.assignments[bot_id]][0] == 1:
                    self.arm_control(bot_id , 130 , 35)
                
                self.state_idx_0 += 1
                self.time_0_1 = None
                self.time_0_2 = None
        elif(self.states_order[self.state_idx_0] == 'LOOK_FOR_NEXT'):
            with self.lock:
                if len(self.remaining_tasks) != 0:
                    task_assigned = None
                    min_dist = None
                    for task in self.remaining_tasks:
                        dist = self.cost_function(self.robots[bot_id][:2] ,self.tasks[task][:2])
                        if min_dist is None or dist < min_dist:
                            min_dist = dist
                            task_assigned = task
                    self.assignments[bot_id] = task_assigned
                    self.allocate_drop_location()
                    self.state_idx_0 = 0
                    self.remaining_tasks.remove(task_assigned)

                else:
                    self.state_idx_0 += 1



        

    def update_2(self , bot_id = 2 , thres = 215):


        if(self.state_idx_2 >= len(self.states_order)):
            self.get_logger().info("Mission Accomplished !")
            self.arm_control(bot_id=2)
            return
        elif(self.states_order[self.state_idx_2] == 'IDLE'):
            with self.task_lock:
                if self.task_allocated is False:
                    if not (len(self.robots) == self.num_robots and len(self.tasks) == self.num_crates):
                        print(len(self.robots) , len(self.tasks))
                        return
                    self.assignments = self.greedy_allocation(self.robots , self.tasks)
                    self.allocate_drop_location()
                    print(f"\n\n\n Drop_Locations : {self.drop_allocated}")
                    self.task_allocated = True

            if(self.robot_2_planned is False):    
                self.obstacle_grid_2  = np.ones(shape = (61,61))*5
                self.update_grid(self.obstacle_grid_2 , bot_id)
                print(f"{bot_id} : {self.time_step}")
                tx,ty = self.calculate_pickup_pose(self.tasks[self.assignments[bot_id]][0],self.tasks[self.assignments[bot_id]][1],self.tasks[self.assignments[bot_id]][2],thres)
                print(f"Bot_id : {bot_id} , crate_id : {self.assignments[bot_id]} , Pose : {self.tasks[self.assignments[bot_id]]}")

                with self.planning_lock:
                    self.robot_2_goals = astar_time(self.obstacle_grid_2 , self.robots[bot_id][:2] , [tx,ty],self.reserved_vertices,self.reserved_vertices,bot_id,start_time_step=self.time_step)
                    if(self.robot_2_goals is not None):

                        
                        reserve_path(self.robot_2_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                        self.robot_2_goals[-1] = (tx,ty,self.robot_2_goals[-1][2])
                        print(f"Bot : {bot_id} \n Goals  {self.robot_2_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                        self.robot_2_planned = True
            else:
                self.state_idx_2 += 1
                self.robot_2_planned = False
        
        elif(self.states_order[self.state_idx_2] == 'MOVING'):
            if(self.controller_2 is None and self.robot_2_goals is not None):
                self.arm_control(bot_id)
                if(self.timer_start is None):
                    with self.lock:
                        self.timer_start = self.get_clock().now().nanoseconds/1e9
                if(self.state_idx_2 == 1):
                    self.controller_2 = HolonomicPIDController(bot_id , 10  , assignment = self.assignments[2])
                elif(self.state_idx_2 == 4):
                    drop_zone,yaw = self.give_drop_location(bot_id , self.assignments[bot_id] , thres )
                    self.controller_2 = HolonomicPIDController(bot_id , 10 , drop_angle=yaw , drop = True)
                else:
                    self.controller_2 = HolonomicPIDController(bot_id , dock = True)
                
                world_goals = []
                if(len(self.robot_2_goals) >= 2):
                    self.robot_2_goals.pop(0)
                for goal in self.robot_2_goals:
                    world_goals.append((goal[0]*40 + 20 , goal[1]*40 + 20 , goal[0]))
                if(self.state_idx_2 == 1 or self.state_idx_2 == 4):
                    world_goals[-1] = (world_goals[-1][0] - 20.0)/40.0,(world_goals[-1][1] - 20.0)/40.0,world_goals[-1][2]
                self.controller_2.set_goals(world_goals)

                with self.lock:
                    self.executor.add_node(self.controller_2)


            elif(self.controller_2 is not None ):
                with goals_reached_lock_2:
                    if self.controller_2.all_goals_reached is False:
                        return
                with self.lock:
                    self.executor.remove_node(self.controller_2)
                #self.controller_2.destroy_node()
                self.controller_2 = None
                self.state_idx_2 += 1

            
        elif(self.states_order[self.state_idx_2] == 'DROP_PLANNING'):
            if self.drop_allocated[self.assignments[bot_id]][0] == 0:
                drop_zone,yaw = self.give_drop_location(bot_id , self.assignments[bot_id] , thres)
            else:
                drop_zone,yaw = self.give_drop_location(bot_id , self.assignments[bot_id] , thres - 15.0)

            if(self.robot_2_planned is False):
                self.obstacle_grid_2 = np.ones(shape = (61,61))*5
                self.update_grid(self.obstacle_grid_2 , bot_id)
                print(f"{bot_id} : {self.time_step}")

                with self.planning_lock:
                    self.robot_2_goals = astar_time(self.obstacle_grid_2 , self.robots[bot_id][:2] , drop_zone ,self.reserved_vertices,self.reserved_vertices,bot_id,start_time_step=self.time_step)
                    
                    if(self.robot_2_goals is not None):
                        reserve_path(self.robot_2_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                        self.robot_2_goals[-1] = (drop_zone[0],drop_zone[1],self.robot_2_goals[-1][2])
                        print(f"Bot : {bot_id} \n Goals  {self.robot_2_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                    
                        self.robot_2_planned = True

            else:
                self.state_idx_2 += 1
                self.robot_2_planned = False


        elif(self.states_order[self.state_idx_2] == 'DOCK_PLANNING'):
            if(self.robot_2_planned is False):
                self.obstacle_grid_2 = np.ones(shape = (61,61))*5
                self.update_grid(self.obstacle_grid_2 , bot_id)

                print(f"{bot_id} : {self.time_step}")
                with self.planning_lock:
                    self.robot_2_goals = astar_time(self.obstacle_grid_2 , self.robots[bot_id][:2] , self.dock_2 ,self.reserved_vertices,self.reserved_vertices,bot_id,start_time_step=self.time_step)
                    
                    if(self.robot_2_goals is not None):

                        reserve_path(self.robot_2_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                        print(f"Bot : {bot_id} \n Goals  {self.robot_2_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                        
                        self.robot_2_planned = True

            else:
                self.state_idx_2 += 1
                self.robot_2_planned = False

        elif(self.states_order[self.state_idx_2] == 'PICKUP'):
            if(self.time_2_1 is None):
                self.time_2_1 = self.get_clock().now().nanoseconds
                self.arm_control(bot_id , 65 , 0)
                self.arm_control(bot_id , 55 , 0)

            elif(self.get_clock().now().nanoseconds - self.time_2_1 > 2e9 and self.time_2_2 is None):
                self.time_2_2 = self.get_clock().now().nanoseconds
                self.send_attach_request(bot_id = 2)
            
            elif(self.time_2_2 is not None and self.get_clock().now().nanoseconds - self.time_2_2 > 2e9):

                self.arm_control(bot_id , 80 , 0)
                self.arm_control(bot_id , 90 , 0)
                self.arm_control(bot_id , 100 , 0)
                self.state_idx_2 += 1
                self.time_2_1 = None
                self.time_2_2 = None

        elif(self.states_order[self.state_idx_2] == 'DROP'):
            if(self.time_2_1 is None):
                self.time_2_1 = self.get_clock().now().nanoseconds
                if self.drop_allocated[self.assignments[bot_id]][0] == 0:
                    self.arm_control(bot_id , 120,0)
                    self.arm_control(bot_id , 65 , 0)
                    self.arm_control(bot_id , 60 , 0)
                elif self.drop_allocated[self.assignments[bot_id]][0] == 1:
                    self.arm_control(bot_id , 90 , 25)
                

            elif(self.get_clock().now().nanoseconds - self.time_2_1 > 2e9 and self.time_2_2 is None):
                self.time_2_2 = self.get_clock().now().nanoseconds
                self.send_detach_request(bot_id = 2)
            
            elif(self.time_2_2 is not None and self.get_clock().now().nanoseconds - self.time_2_2 > 2e9):
                if self.drop_allocated[self.assignments[bot_id]][0] == 0:
        
                    self.arm_control(bot_id , 130 , 0)
                elif self.drop_allocated[self.assignments[bot_id]][0] == 1:
                    self.arm_control(bot_id , 130 , 0)
                self.state_idx_2 += 1
                self.time_2_1 = None
                self.time_2_2 = None
        
        elif(self.states_order[self.state_idx_2] == 'LOOK_FOR_NEXT'):
            with self.lock:
                if len(self.remaining_tasks) != 0:
                    task_assigned = None
                    min_dist = None
                    for task in self.remaining_tasks:
                        dist = self.cost_function(self.robots[bot_id][:2] ,self.tasks[task][:2])
                        if min_dist is None or dist < min_dist:
                            min_dist = dist
                            task_assigned = task
                    self.assignments[bot_id] = task_assigned
                    self.allocate_drop_location()
                    self.state_idx_2 = 0
                    self.remaining_tasks.remove(task_assigned)
                else:
                    self.state_idx_2 += 1

                

    def update_4(self , bot_id = 4 , thres = 215):
        if(self.state_idx_4 >= len(self.states_order)):
            self.get_logger().info("Mission Accomplished !")
            self.arm_control(bot_id=bot_id)
            return
        elif(self.states_order[self.state_idx_4] == 'IDLE'):
            with self.task_lock:
                if self.task_allocated is False:
                    if not (len(self.robots) == self.num_robots and len(self.tasks) == self.num_crates):
                        print(len(self.robots) , len(self.tasks))
                        return
                    self.assignments = self.greedy_allocation(self.robots , self.tasks)
                    self.allocate_drop_location()
                    print(f"\n\n\n Drop_Locations : {self.drop_allocated}")
                    self.task_allocated = True

            if(self.robot_4_planned is False):    
                self.obstacle_grid_4  = np.ones(shape = (61,61))*5
                self.update_grid(self.obstacle_grid_4 , bot_id)

                print(f"{bot_id} : {self.time_step}")
                tx,ty = self.calculate_pickup_pose(self.tasks[self.assignments[bot_id]][0],self.tasks[self.assignments[bot_id]][1],self.tasks[self.assignments[bot_id]][2],thres)
                print(f"Bot_id : {bot_id} , crate_id : {self.assignments[bot_id]} , Pose : {self.tasks[self.assignments[bot_id]]}")
                with self.planning_lock:
                    self.robot_4_goals = astar_time(self.obstacle_grid_4 , self.robots[bot_id][:2] , [tx,ty],self.reserved_vertices,self.reserved_vertices,bot_id,start_time_step=self.time_step)
                    if(self.robot_4_goals is not None):

                        reserve_path(self.robot_4_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                        self.robot_4_goals[-1] = (tx,ty,self.robot_4_goals[-1][2])
                        print(f"Bot : {bot_id} \n Goals  {self.robot_4_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                        self.robot_4_planned = True
            else:
                self.state_idx_4 += 1
                self.robot_4_planned = False
    
        elif(self.states_order[self.state_idx_4] == 'MOVING'):
            if(self.controller_4 is None and self.robot_4_goals is not None):
                self.arm_control(bot_id)
                if(self.timer_start is None):
                    with self.lock:
                        self.timer_start = self.get_clock().now().nanoseconds/1e9
                if(self.state_idx_4 == 1):
                    self.controller_4 = HolonomicPIDController(bot_id , 10  , assignment = self.assignments[4])
                elif(self.state_idx_4 == 4):
                    drop_zone,yaw = self.give_drop_location(bot_id , self.assignments[bot_id] , thres )
                    self.controller_4 = HolonomicPIDController(bot_id , 10 , drop_angle=yaw , drop = True)
                else:
                    self.controller_4 = HolonomicPIDController(bot_id , dock = True)

                
                if(len(self.robot_4_goals) >= 2):
                    self.robot_4_goals.pop(0)
                world_goals = []
                for goal in self.robot_4_goals:
                    world_goals.append((goal[0]*40 + 20 , goal[1]*40 + 20 , goal[2]))
                if(self.state_idx_4 == 1 or self.state_idx_4 == 4):
                    world_goals[-1] = (world_goals[-1][0] - 20.0)/40.0,(world_goals[-1][1] - 20.0)/40.0,world_goals[-1][2]
                self.controller_4.set_goals(world_goals)

                with self.lock:
                    self.executor.add_node(self.controller_4)


            elif(self.controller_4 is not None):
                with goals_reached_lock_4:
                    if self.controller_4.all_goals_reached is False:
                        return
                    
                with self.lock:
                    self.executor.remove_node(self.controller_4)
                #self.controller_4.destroy_node()
                self.controller_4 = None
                self.state_idx_4 += 1

            
        elif(self.states_order[self.state_idx_4] == 'DROP_PLANNING'):

            if self.drop_allocated[self.assignments[bot_id]][0] == 0:
                drop_zone,yaw = self.give_drop_location(bot_id , self.assignments[bot_id] , thres)
            else:
                drop_zone,yaw = self.give_drop_location(bot_id , self.assignments[bot_id] , thres - 10.0)

            if(self.robot_4_planned is False):
                self.obstacle_grid_4 = np.ones(shape = (61,61))*5
                self.update_grid(self.obstacle_grid_4 , bot_id)

                print(f"{bot_id} : {self.time_step}")
                with self.planning_lock:
                    self.robot_4_goals = astar_time(self.obstacle_grid_4 , self.robots[bot_id][:2] , drop_zone ,self.reserved_vertices,self.reserved_vertices,bot_id,start_time_step=self.time_step)

                    if(self.robot_4_goals is not None):

                        reserve_path(self.robot_4_goals , self.reserved_vertices , self.reserved_edges , bot_id)
                        self.robot_4_goals[-1] = (drop_zone[0],drop_zone[1],self.robot_4_goals[-1][2])
                        print(f"Bot : {bot_id} \n Goals  {self.robot_4_goals} \n Reserved Edges : {self.reserved_edges} \n Reserved Vertices : {self.reserved_vertices} ")
                    
                        self.robot_4_planned = True

            else:
                self.state_idx_4 += 1
                self.robot_4_planned = False


        elif(self.states_order[self.state_idx_4] == 'DOCK_PLANNING'):
            if(self.robot_4_planned is False):
                self.obstacle_grid_4 = np.ones(shape = (61,61))*5
                self.update_grid(self.obstacle_grid_4 , bot_id)

                print(f"{bot_id} : {self.time_step}")
                with self.planning_lock:
                    self.robot_4_goals = astar_time(self.obstacle_grid_4 , self.robots[bot_id][:2] , self.dock_4 ,self.reserved_vertices,self.reserved_vertices,bot_id,start_time_step=self.time_step)
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
                self.arm_control(bot_id , 60 , 5)
                self.arm_control(bot_id , 50 , 5)

            elif(self.get_clock().now().nanoseconds - self.time_4_1 > 2e9 and self.time_4_2 is None):
                self.time_4_2 = self.get_clock().now().nanoseconds
                self.send_attach_request(bot_id = 4)
            
            elif(self.time_4_2 is not None and self.get_clock().now().nanoseconds - self.time_4_2 > 2e9):
                self.arm_control(bot_id , 130 , 5)
                

                self.state_idx_4 += 1
                self.time_4_1 = None
                self.time_4_2 = None

        elif(self.states_order[self.state_idx_4] == 'DROP'):
            if(self.time_4_1 is None):
                self.time_4_1 = self.get_clock().now().nanoseconds
                if self.drop_allocated[self.assignments[bot_id]][0] == 0:
                    self.arm_control(bot_id , 60 , 5)
                    self.arm_control(bot_id , 50 , 5)
                elif self.drop_allocated[self.assignments[bot_id]][0] == 1:
                    self.arm_control(bot_id , 85 , 30)

            elif(self.get_clock().now().nanoseconds - self.time_4_1 > 2e9 and self.time_4_2 is None):
                self.time_4_2 = self.get_clock().now().nanoseconds
                self.send_detach_request(bot_id = 4)
            
            elif(self.time_4_2 is not None and self.get_clock().now().nanoseconds - self.time_4_2 > 2e9):
                if self.drop_allocated[self.assignments[bot_id]][0] == 0:
        
                    self.arm_control(bot_id , 90 , 5)
                elif self.drop_allocated[self.assignments[bot_id]][0] == 1:
                    self.arm_control(bot_id , 130 , 30)
                self.state_idx_4 += 1
                self.time_4_1 = None
                self.time_4_2 = None   

        elif(self.states_order[self.state_idx_4] == 'LOOK_FOR_NEXT'):
            with self.lock:
                if len(self.remaining_tasks) != 0:
                    task_assigned = None
                    min_dist = None
                    for task in self.remaining_tasks:
                        dist = self.cost_function(self.robots[bot_id][:2] ,self.tasks[task][:2])
                        if min_dist is None or dist < min_dist:
                            min_dist = dist
                            task_assigned = task
                    self.assignments[bot_id] = task_assigned
                    self.allocate_drop_location()
                    self.state_idx_4 = 0
                    self.remaining_tasks.remove(task_assigned)
                else:
                    self.state_idx_4 += 1

                  

            

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
            for j in range(35):
                f_surr(x,y,t+j)
         
        if i>0:
            px,py,pt = path[i-1]
            reserved_edges[(x,y,px,py,t)] = robot_id           

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = fsm(exec = executor )
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

            


            




            


        
            

            

            


