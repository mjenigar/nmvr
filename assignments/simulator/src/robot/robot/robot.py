import sys
import rclpy
import math

import numpy as np

from rclpy.node import Node
# from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from regulator import PID

class Robot(Node):
    def __init__(self, pos, name="Isaac"):
        super().__init__(name)
        self.name = name
        self.size = 12
        self.odo = Odometry()
        self.odo.pose.pose.position.x = float(pos[0])
        self.odo.pose.pose.position.y = float(pos[1])
        ### Status
        self.status = "Searching for connection"
        self.velocity = np.array([0.01, 0])
        self.connected = False
        self.change = True

        ### Publishers 
        self.pub_vel = self.create_publisher("cmd_vel".format(self.name), Twist, 10)
        ### Listeners
        self.world_conn_listener = self.create_subscription(String, "world_connection", self.Connect2World, 10)
        self.ping_response = self.create_subscription(String, "world_ping", self.ResponsePing, 10)
        self.pose_listener = self.create_subscription("robot_position", Pose, self.UpdatePose)
        
        ### Timers
        self.search4world = self.create_timer(1, self.Search4World)
        self.robot_status = self.create_timer(1, self.GetStatus)

        self.angle_PID = PID(1.4, 0, 0)
        self.distance_PID = PID(1.4, 0, 0)
        
        self.goal_x = 5.0
        self.goal_y = 0.0
        
    def GetStatus(self):
        if self.change:
            self.get_logger().info("STATUS: {} CONN: {} POSE x: {} y: {}".format(self.status, self.connected, self.odo.pose.pose.position.x, self.odo.pose.pose.position.y))
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
        # self.destroy_publisher(self.conn_req)
    
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

    def AngControl(self):
        self.R = math.sqrt(math.pow(self.current_pose_x - self.goal_x , 2) + math.pow(self.current_pose_y - self.goal_y , 2))

        self.xr = self.R*math.cos(self.current_angle)
        self.yr = self.R*math.sin(self.current_angle)

        self.xim = self.current_pose_x + self.xr
        self.yim = self.current_pose_y + self.yr

        self.C = math.sqrt(math.pow(self.xim - self.goal_x , 2) + math.pow(self.yim - self.goal_y , 2))

        if self.xim > self.goal_x:
            self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
        else:
            self.alpha = 2 * math.pi * math.acos((2 * math.pow(self.R, 2) - math.pow(self.C, 2)) / (2 * math.pow(self.R, 2)))
        
        while self.alpha > 0.005: 
            self.R = math.sqrt(math.pow(self.current_pose_x - self.goal_x , 2) + math.pow(self.current_pose_y - self.goal_y , 2))
            self.xr = self.R*math.cos(self.current_angle)
            self.yr = self.R*math.sin(self.current_angle)

            self.xim = self.current_pose_x + self.xr
            self.yim = self.current_pose_y + self.yr

            self.C = math.sqrt(math.pow(self.xim - self.goal_x , 2) + math.pow(self.yim - self.goal_y , 2))
            
            if self.xim > self.goal_x:
                self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
            else:
                self.alpha = 2*3.14*math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))

            self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
            self.PID_angle = self.angle_PID.update(self.alpha)
            self.msg.angular.z = self.PID_angle
            self.pub_vel.publish(self.msg)
    
    def distance_controller(self):
        self.distance = math.sqrt(math.pow(self.goal_x - self.current_pose_x , 2) + math.pow(self.goal_y - self.current_pose_y, 2 ))
        print("distance: " + str(self.distance))
        while self.distance > 0.15:
            self.distance = math.sqrt(math.pow(self.goal_x - self.current_pose_x , 2) + math.pow(self.goal_y - self.current_pose_y, 2 ))
            self.PID_distance = self.distance_PID.update(self.distance)
            self.msg.linear.x = self.PID_distance
            self.pub_vel.publish(self.msg)
    
    def UpdatePose(self, data):
        self.current_pose_x = data.x
        self.current_pose_y = data.y
        self.current_angle = data.theta
    
def main():
    rclpy.init()

    robot = Robot([0,0])
    rclpy.spin(robot)
    
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
