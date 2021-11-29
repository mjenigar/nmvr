import sys
import rclpy
import math
import time 
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# from regulator import PID

class Robot(Node):
    def __init__(self, pos, name="Isaac"):
        super().__init__(name)
        ### Status
        self.status = "Searching for connection"
        self.connected = False
        self.change = True
        self.name = name
        self.size = 32
        self.wheel_r = 0.5

        ### Init position
        self.odo = Odometry()
        self.odo.pose.pose.position.x = float(pos[0])
        self.odo.pose.pose.position.y = float(pos[1])
        self.odo.pose.pose.orientation = self.quaternion_from_euler(0,0,0)
        
        ### Publishers 
        self.pub_position = self.create_publisher(Pose, "current_pose", 10)

        ### Listeners
        self.world_conn_listener = self.create_subscription(String, "world_connection", self.Connect2World, 10)
        self.ping_response = self.create_subscription(String, "world_ping", self.ResponsePing, 10)
        # self.pose_listener = self.create_subscription(Pose, "robot_position", self.UpdatePose, 10)
        
        ### Timers
        self.search4world = self.create_timer(1, self.Search4World)
        self.robot_status = self.create_timer(1, self.GetStatus)

        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        
        self.angle_PID = PID(1.4, 0, 0) 
        self.distance_PID = PID(0.3, 0, 0)
        
        ### TMP
        # TODO create input from GUI
        self.SetGoal(20, 3)
        
    def GetStatus(self):
        if self.change:
            self.get_logger().info("STATUS: {} CONN: {} POSE x: {} y: {} theta: {}".format(self.status, self.connected, self.odo.pose.pose.position.x, self.odo.pose.pose.position.y, self.odo.pose.pose.orientation.z))
            self.change = False

    def PublishStrMsg(self, topic, _msg, log=True):
        pub = self.create_publisher(String, topic, 10)
        msg = String()
        msg.data = _msg
        pub.publish(msg)
        if log:
            self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(msg.data), msg.data))
        
        self.destroy_publisher(pub)
    
    def Search4World(self):
        self.conn_req = self.create_publisher(String, "connection_req", 10)
        msg = String()
        msg.data = "{}_{}-{}".format(self.name, self.odo.pose.pose.position.x, self.odo.pose.pose.position.y)
        self.conn_req.publish(msg)
        # self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(msg.data), msg.data))
    
    def Connect2World(self, msg):
        response = msg.data.split("_")[0]
        if response == "true":
            self.connected = True
            self.change = True
            self.status = "idle"
            self.search4world.destroy()
            self.get_logger().info("CONNECTED!")
            self.destroy_publisher(self.conn_req)

            # ### Move
            self.Move2Goal()
        else:
            self.get_logger().info("CONNECTION REFUSED!: {} REASON: {}".format(response, msg.data.split("_")[1]))

    def ResponsePing(self, msg):
        self.PublishStrMsg("ping_response", "robot_{}".format(self.name, self.connected), False)

    ### Move ####
    def SetGoal(self, x, y):
        self.goal = Pose()
        self.goal.position.x = float(x) 
        self.goal.position.y = float(y) 
        
    def Move2Goal(self, tolerance=1.5):
        next_pose = Pose()
        distance = self.GetDistance()
        # linear px/sec
        self.odo.twist.twist.linear.x = 0.1
        self.odo.twist.twist.linear.y = 0.0
        self.odo.twist.twist.linear.z = 0.0
        # angular rad/sec
        self.odo.twist.twist.angular.x = 0.0
        self.odo.twist.twist.angular.y = 0.0
        self.odo.twist.twist.angular.z = 0.1
        
        while distance >= tolerance:
            self.current_time = self.get_clock().now()
            self.dt = (self.current_time - self.last_time).nanoseconds / 1e9
            distance = self.GetDistance()
            print("Distance from goal: {}".format(distance))

            v_left = self.wheel_r * self.odo.twist.twist.linear.x
            v_right = self.wheel_r * self.odo.twist.twist.linear.x
            d_left = v_left * self.dt
            d_right = v_right * self.dt
            
            d_center = (d_left + d_right)/2
            phi = (d_right - d_left)/self.size - 4
            
            self.odo.twist.twist.linear.x = self.distance_PID.update(distance/self.dt)
            print("{} px/sec".format(self.odo.twist.twist.linear.x))
            self.odo.twist.twist.angular.z = self.angle_PID.update(phi/self.dt)
            print("{} rad/sec".format(self.odo.twist.twist.angular.z))
            
            next_pose.position.x = self.odo.pose.pose.position.x + d_center*math.cos(self.odo.pose.pose.orientation.z)
            next_pose.position.y = self.odo.pose.pose.position.y + d_center*math.sin(self.odo.pose.pose.orientation.z)               
            print(next_pose.position.y)
            
            next_pose.orientation = self.quaternion_from_euler(0,0, float(self.odo.pose.pose.orientation.z + phi))
            next_pose = self.Edges(next_pose)
            
            self.pub_position.publish(next_pose)
            print("x:{} y:{} z:{} deg\n\n".format(next_pose.position.x, next_pose.position.y, next_pose.orientation.z))            
            self.UpdatePose(next_pose)
            time.sleep(1)
        
        print("Stop")
        self.odo.twist.twist.linear.x = 0.0
        self.odo.twist.twist.angular.z = 0.0
        
    # def GetNextPosition(self):
    #     next_pose = Pose()
        
    #     v_left = self.wheel_r * self.vel.angular.z 
    #     v_right = self.wheel_r * self.vel.angular.z
    #     d_left = v_left * self.dt
    #     d_right = v_right * self.dt
        
    #     d_center = (d_left + d_right)/2
    #     phi = (d_right - d_left)/self.size
        
    #     next_pose.orientation.z = self.odo.pose.pose.orientation.z + phi
    #     next_pose.position.x = self.odo.pose.pose.position.x + d_center*math.cos(phi) 
    #     next_pose.position.y = self.odo.pose.pose.position.y + d_center*math.sin(phi)                

        # return self.Edges(next_pose)

    def UpdatePose(self, next_pose):
        self.odo.pose.pose.position.x = next_pose.position.x 
        self.odo.pose.pose.position.y = next_pose.position.y
        self.odo.pose.pose.orientation.z = next_pose.orientation.z

    def Edges(self, next_pose):
        if next_pose.position.x > 38:
            next_pose.position.x = 38.0 
        # elif next_pose.position.x < 0:
        #     next_pose.position.x = 0.0 
        
        if next_pose.position.y > 38:
            next_pose.position.y = 38.0
        # elif next_pose.position.y < 0:
        #     next_pose.position.y = 0.0
        
        return next_pose    
    
    ### Distance ###
    def GetDistance(self, method="euclidean"):
        if method == "euclidean":
            return math.sqrt(math.pow((self.goal.position.x - self.odo.pose.pose.position.x), 2) +
                    math.pow((self.goal.position.y - self.odo.pose.pose.position.y), 2))
        elif method == "manhathan":
            return (self.goal.position.x - self.odo.pose.pose.position.x) + (self.goal.position.y - self.odo.pose.pose.position.y)
        else:
            return None
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        SRC: https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
        """
        q = Quaternion()
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z =sy * cp * cr - cy * sp * sr
        
        return q
    
    def GetAlpha(self, tolerance=0.005):
        current_x = self.odo.pose.pose.position.x
        current_y = self.odo.pose.pose.position.y 
        goal_x = self.goal.position.x 
        goal_y = self.goal.position.y 
        theta =  self.odo.pose.pose.orientation.z
        
        self.R = math.sqrt(math.pow(current_x - goal_x , 2) + math.pow(current_y - goal_y , 2))
        self.xr = self.R*math.cos(theta)
        self.yr = self.R*math.sin(theta)
        self.xim = current_x + self.xr
        self.yim = current_y + self.yr
        self.C = math.sqrt(math.pow(self.xim - goal_x , 2) + math.pow(self.yim - goal_y , 2))
        if self.xim > goal_x:
            return math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
        else:
            return 2 * math.pi * math.acos((2 * math.pow(self.R, 2) - math.pow(self.C, 2)) / (2 * math.pow(self.R, 2)))






class PID:
    def __init__(self, P, I, D,  Derivator=0, Integrator=0, IntegratorRange=[-500, 500]):
        self.kp = P 
        self.ki = I 
        self.kd = D
        self.derivator = Derivator
        self.integrator = Integrator
        self.integratorRange = IntegratorRange
        
        self.goal = 0.0
        self.error = 0.0
        
    def update(self, error):
        self.error = error
        
        self.P = self.kp * self.error
        self.D = self.kd * (self.error - self.derivator)
        self.derivator = self.error
        
        self.integrator += self.error
        if self.integrator > max(self.integratorRange):
            self.integrator = max(self.integratorRange)
        elif self.integrator < min(self.integratorRange):
            self.integrator = min(self.integratorRange)
            
        self.I = self.integrator * self.ki
        
        return self.P + self.I + self.D
    
    def setGoal(self, goal):
        self.goal = goal
        self.integrator = 0
        self.derivator = 0
        
def main():
    rclpy.init()

    robot = Robot([1,0])
    rclpy.spin(robot)
    
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
