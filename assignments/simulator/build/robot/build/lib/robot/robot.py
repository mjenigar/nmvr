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
        self.wheel_r = 1.0
        self.baseline = 1.0
        self.max_lin_vel = 5.0
        self.max_ang_vel = math.pi

        ### Init position
        self.odo = Odometry()
        self.odo.pose.pose.position.x = float(pos[0])
        self.odo.pose.pose.position.y = float(pos[1])
        self.odo.pose.pose.orientation.z = 0.0
        
        ### Publishers 
        self.pub_position = self.create_publisher(Odometry, "current_pose", 10)
        self.pub_dist = self.create_publisher(String, "current_distance", 10)
        
        ### Listeners
        self.world_conn_listener = self.create_subscription(String, "world_connection", self.Connect2World, 10)
        self.ping_response = self.create_subscription(String, "world_ping", self.ResponsePing, 10)
        self.respawn_listener = self.create_subscription(Pose, "robot_respawn", self.UpdatePose, 10)
        self.move2goal_listener = self.create_subscription(Pose, "move_to_goal", self.StartMove, 10)
        
        ### Timers
        self.search4world = self.create_timer(1, self.Search4World)
        self.robot_status = self.create_timer(1, self.GetStatus)

        self.angle_PID = PID(0.42, 0, 0) 
        self.distance_PID = PID(0.33, 0, 0)
        
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
        else:
            self.get_logger().info("CONNECTION REFUSED!: {} REASON: {}".format(response, msg.data.split("_")[1]))

    def ResponsePing(self, msg):
        self.PublishStrMsg("ping_response", "robot_{}".format(self.name, self.connected), False)

    ### Move ####
    def SetGoal(self, x, y):
        self.goal = Pose()
        self.goal.position.x = float(x) 
        self.goal.position.y = float(y) 
    
    def steering_angle(self):
        return math.atan2(self.goal.position.y - self.odo.pose.pose.position.y, self.goal.position.x - self.odo.pose.pose.position.x)
    
    def StartMove(self, msg):
        self.SetGoal(msg.position.x, msg.position.y)
        self.Move2Goal()
    
    def Move2Goal(self, tolerance=0.5):
        msg = String()
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
            distance = self.GetDistance()
            print("Distance from goal: {}".format(distance))
            d_left = (self.odo.twist.twist.linear.x - 1/2 * self.wheel_r * self.odo.twist.twist.angular.z) * 0.1
            d_right = (self.odo.twist.twist.linear.x + 1/2 * self.wheel_r * self.odo.twist.twist.angular.z) * 0.1
            d_center = (d_left + d_right)/2
            phi = (d_right - d_left) / self.baseline
            
            self.odo.twist.twist.linear.x = self.distance_PID.update(distance)
            print("{} px/sec".format(self.odo.twist.twist.linear.x))
            
            self.odo.twist.twist.angular.z = self.angle_PID.update(self.steering_angle() - self.odo.pose.pose.orientation.z)
            print("{} rad/sec".format(self.odo.twist.twist.angular.z))
            
            next_pose.position.x = self.odo.pose.pose.position.x + d_center*math.cos(self.odo.pose.pose.orientation.z)
            next_pose.position.y = self.odo.pose.pose.position.y + d_center*math.sin(self.odo.pose.pose.orientation.z)               
            next_pose.orientation.z = self.odo.pose.pose.orientation.z + phi
            # next_pose = self.Edges(next_pose)
            
            # print("x:{} y:{} z:{} deg\n\n".format(next_pose.position.x, next_pose.position.y, next_pose.orientation.z))            
            self.UpdatePose(next_pose)
            self.pub_position.publish(self.odo)
            msg.data = str(distance)
            self.pub_dist.publish(msg)
            time.sleep(0.01)
            
            if distance > 100:
                break
        
        print("Stop")
        self.odo.twist.twist.linear.x = 0.0
        self.odo.twist.twist.angular.z = 0.0
        self.pub_position.publish(self.odo)

    def UpdatePose(self, next_pose):
        self.odo.pose.pose.position.x = next_pose.position.x 
        self.odo.pose.pose.position.y = next_pose.position.y
        self.odo.pose.pose.orientation.z = next_pose.orientation.z

    def Edges(self, next_pose):
        if next_pose.position.x > 38:
            next_pose.position.x = 38.0 
        elif next_pose.position.x < 0:
            next_pose.position.x = 0.0 
        
        if next_pose.position.y > 38:
            next_pose.position.y = 38.0
        elif next_pose.position.y < 0:
            next_pose.position.y = 0.0
        
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
    
class PID:
    def __init__(self, P, I, D,  Derivator=0, Integrator=0, IntegratorRange=[-500, 500]):
        self.kp = P 
        self.ki = I 
        self.kd = D
        self.derivator = Derivator
        self.integrator = Integrator
        self.integratorRange = IntegratorRange
        
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
        
def main():
    rclpy.init()

    robot = Robot([2, 2])
    rclpy.spin(robot)
    
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
