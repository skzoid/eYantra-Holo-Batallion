#!/usr/bin/env python3

'''
This Python file runs a ROS 2 node of name holonomic_pid_controller which holds the position of a holonomic robot
and drives it through a series of predefined goals using PID controllers on [x, y, θ].

This node publishes and subscribes to the following topics:

        PUBLICATIONS                               SUBSCRIPTIONS
        /forward_velocity_controller/commands      /bot_pose

Instead of defining separate variables for each PID axis, lists/dictionaries are used.
For example: pid_params['x'], pid_params['y'], pid_params['theta'], etc.

Code modularity and clarity are maintained to make tuning and extension easier.
'''

# ---------------------- Import Required Librari---------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import time

# import hb_interface messages
from hb_interfaces.msg import Poses2D

import numpy as np
import math
from hb_interfaces.msg import BotCmd
from hb_interfaces.msg import BotCmdArray
# from linkattacher_msgs.10.214.179.126srv import AttachLink
# from linkattacher_msgs.srv import DetachLink  


# ---------------------- PID Controller Class --------------------------------
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


# ---------------------- Main Node Class -------------------------------------
class HolonomicPIDController(Node):
    def __init__(self , bot_id , final_threshold = 10.0 , dock_or_drop = True , assignment = None):
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
        self.crate_id = None
        self.start_time = None
        self.time_set = False
        self.assignment = assignment

        # 3. Goal tracking index
        self.goal_index = 0
        self.flag = 0

        # 4. Timing information:
        #    - Used to calculate the time difference (dt) between control loop iterations.
        self.last_time = self.get_clock().now()

        # 5. Threshold for goal completion:
        #    - Defines the acceptable error tolerance for x, y, and θ.
        #    - Example: if error < 5 units → goal considered reached.
        self.goal_threshold = {'pos': 15.0, 'theta': 0.1}
        self.final_threshold = final_threshold
        self.attached = False
        self.detached = False
        self.goals = []
        self.target_angle = None
        self.target_flag = False
        self.drop_or_dock = dock_or_drop


        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 2.5, 'ki': 0.00, 'kd': 0.1, 'max_out': self.max_vel},
            'y': {'kp': 2.5, 'ki': 0.00, 'kd': 0.1, 'max_out': self.max_vel},
            'theta': {'kp': 0.5, 'ki': 0.00, 'kd': 0.1, 'max_out': self.max_vel}
        }

        # Initialize PIDs
        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        # ---------------- ROS 2 Publishers & Subscribers ----------------
        
        # Write a subscriber for /bot_pose
        self.pose_sub = self.create_subscription(
            Poses2D,
            '/bot_pose',
            self.pose_cb,
            10
        )

        self.publisher = self.create_publisher(
            BotCmdArray, '/bot_cmd', 100
        )
        
        self.crate_pose_sub = self.create_subscription(
            Poses2D,
            'crate_pose',
            self.crate_pose_cb,
            10
        )
        if(bot_id == 0):
            self.sensor_subscription = self.create_subscription(Bool , '/sensor0' ,self.sensor_callback , 10)
        elif(bot_id == 2):
            self.sensor_subscription = self.create_subscription(Bool , '/sensor2' ,self.sensor_callback , 10)
        elif(bot_id == 4):
            self.sensor_subscription = self.create_subscription(Bool , '/sensor4' ,self.sensor_callback , 10)

        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.05, self.control_cb)  # ~30ms = 33 Hz
        self.wait_start = self.get_clock().now()
        self.waiting = False

        self.attach_client = self.create_client(SetBool , '/attach' )
        self.all_goals_reached = False
        self.halt = False
        self.bot_id = bot_id
        self.close_flag = False
        self.detection = False


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
                self.crate_pose = [pose.x, pose.y, yaw_in_radians]
                self.crate_id = pose.id


    def sensor_callback(self , msg):
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
    
    def pause(self):
        self.halt = True

    def resume(self):
        self.halt = False

    
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

        # Check if current pose is available
        if(self.halt is True):
            self.publish_wheel_velocities([0.0,0.0,0.0])
            print(f"{self.bot_id} -- Paused.")
            return
        
        else:
            if(self.time_set is False):
                self.time_set = True
                self.start_time = self.get_clock().now().nanoseconds
                print("-"*15 , f"Start time : {self.start_time}" , "-"*15)
            
            if(self.goal_index >= len(self.goals)):
                self.goal_index -= 1

            if(self.goal_index >= 3/4 * len(self.goals) and self.close_flag is False):
                self.close_flag = True
                self.pid_x.kp = 0.75
                self.pid_y.kp = 0.75
                self.pid_theta.kp = 0.2


            if self.current_pose is None :
                self.get_logger().info(f'Waiting for pose data...{self.robot_id}')
                return

            if len(self.goals) == 0:
                self.get_logger().info(f'Waiting for goals...{self.robot_id}')
                return
            else:
                if(self.assignment is None or self.crate_pose is None):
                    self.final_goal = self.goals[-1]
                else:
                    self.final_goal = self.crate_pose


            

        
            # Time delta
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0:
                return
            self.last_time = now

            # Current robot pose
            x, y, theta = self.current_pose

            

            # If all goals are reached
            # if (self.goal_index >= len(self.goals) ):
            #     self.publish_wheel_velocities([0.0, 0.0, 0.0])
            #     self.all_goals_reached = True
            #     self.get_logger().info('All goals reached! Mission accomplished.')
            #     return

            # Current target goal
            if(self.target_flag is False):
                target_x, target_y = self.goals[self.goal_index]
            else:
                target_x,target_y = self.final_goal[0],self.final_goal[1]


            #target_theta = math.radians(target_theta_deg)
            if(self.drop_or_dock is False):
                target_theta = self.set_target_angle([target_x , target_y] , [x,y]) 
            else:
                target_theta = np.pi/2


            # Errors
            error_x = target_x - x
            error_y = target_y - y

            error_theta = target_theta - theta 
            if(self.bot_id == 4):
                error_theta -= 0.1
            if(self.bot_id == 2):
                error_theta -= 0.05
            # if(error_theta > 2*np.pi):
            #     error_theta -= 2*np.pi
            error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
            # if(self.bot_id == 2):
            #     print(error_theta , f" target_yaw : {target_theta*180/np.pi} Current_yaw : {theta*180/np.pi}" )
            # # print('Errors : ',error_theta , error_x , error_y)
            # print('Pose :' , x,y,theta)

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



            w1 = (1/self.WHEEL_RADIUS) * (math.sin(theta1)*vx_local - math.cos(theta1)*vy_local + (self.ROBOT_RADIUS)*omega)
            w2 = (1/self.WHEEL_RADIUS) * (math.sin(theta2)*vx_local - math.cos(theta2)*vy_local + (self.ROBOT_RADIUS)*omega)
            w3 = (1/self.WHEEL_RADIUS) * (math.sin(theta3)*vx_local - math.cos(theta3)*vy_local + (self.ROBOT_RADIUS)*omega)
            wheel_vel = [w1, w2, w3]



            # Goal check

            #if(pos_error < self.goal_threshold['pos'] and abs(error_theta) < self.goal_threshold['theta'] ):
            dist = self.dist(self.final_goal , [x,y])
            # if(self.bot_id == 0):
            #     print(f"Final Goal : {self.final_goal} --- Current : {x},{y} --- Distance : {dist}")
            if(dist < self.final_threshold):
                self.pid_x.kp = 0.0
                self.pid_x.kd = 0.0
                self.pid_y.kp = 0.0
                self.pid_y.kd = 0.0
                self.pid_theta.kp = 0.5

                if(self.bot_id == 4):
                    if(wheel_vel[0] > 0.05):
                        wheel_vel = [11.0 ,9.0 , 9.0]
                    elif(wheel_vel[0] < -0.05):
                        wheel_vel = [-3.0 , -3.0 , -3.0]

                    

                self.target_flag = True
                print(f"{self.bot_id} -- Stopped!")

                if((abs(error_theta) < 0.2) and ((self.drop_or_dock) or ((not self.drop_or_dock) and self.detection))):
                    print(f"{self.bot_id} -- Goal_Reached -- Detection :{self.detection}")
                    wheel_vel = [0.0 , 0.0 , 0.0]
                    self.all_goals_reached = True
                    now = self.get_clock().now().nanoseconds
                    print("-"*15 , f"End Time : {now} , Time Taken : {now - self.start_time}" , "-"*15)
            elif(self.pid_x.kp == 0):
                self.pid_x.kp = 0.25
                self.pid_y.kp = 0.25
                self.pid_theta.kp = 0.0

            elif(pos_error < self.goal_threshold['pos']):

                self.goal_index += 1
                print(self.bot_id , f": Goal : {self.goal_index}")

            
            self.publish_wheel_velocities(wheel_vel)




    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel , angle = [90.0 , 90.0]):
        # Wheel velocity array (Float64MultiArray)
        # Order: [Left wheel speed, Right wheel speed, Rear wheel speed]
        msg = BotCmd()

        for i in range(len(wheel_vel)):
            if (0.05 < wheel_vel[i]):
                wheel_vel[i] += 7.0
            if(-0.05 > wheel_vel[i]):
                wheel_vel[i] -= 3.0  

        msg.m1 = np.float64((wheel_vel[0]+90))
        msg.m2 = np.float64(wheel_vel[1] + 90)
        msg.m3 = np.float64(wheel_vel[2]+90)
        msg.id = self.bot_id
        msg.elbow = np.float64(angle[1])
        msg.base = np.float64(angle[0])
        Msg = BotCmdArray()
        Msg.cmds = [msg]
        self.publisher.publish(Msg)
        # if(self.bot_id == 0):
        #     print(Msg)


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()