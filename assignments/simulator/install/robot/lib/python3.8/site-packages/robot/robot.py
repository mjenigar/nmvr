import sys
import rclpy

import numpy as np

from rclpy.node import Node
from std_msgs.msg import String

#TODO Create Logger 

class Robot(Node):
    def __init__(self, pos, name="Isaac"):
        super().__init__(name)
        self.name = name
        self.robot = None
        self.size = 12
        
        ######
        ### Status
        self.status = "Searching for connection"
        self.pos = np.array(pos)
        self.velocity = np.array([0.01, 0])
        self.connected = False
        self.change = True
        
        ### Listeners
        self.world_conn_listener = self.create_subscription(String, "world_connection", self.Connect2World, 10)
        self.ping_response = self.create_subscription(String, "world_ping", self.ResponsePing, 10)
        
        # self.forward_listener = self.create_subscription(String, "{}_forward".format(self.name), self.MoveForward, 10)
        # self.backward_listener = self.create_subscription(String, "{}_backward".format(self.name), self.MoveBackward, 10)

        ### Timers
        self.search4world = self.create_timer(1, self.Search4World)
        self.robot_status = self.create_timer(1, self.GetStatus)

    def GetStatus(self):
        if self.change:
            self.get_logger().info("STATUS: {} CONN: {} POS: {} ".format(self.status, self.connected, self.pos))
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
        msg.data = "{}_{}-{}".format(self.name, self.pos[0], self.pos[1])
        self.conn_req.publish(msg)
        # self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(msg.data), msg.data))
        # self.destroy_publisher(self.conn_req)
    
    def Connect2World(self, msg):
        response = msg.data.split("_")[0]
        if response == "true":
            self.connected = True
            self.change = True
            self.status = "waiting for spawn"
            self.search4world.destroy()
            self.get_logger().info("CONNECTED!")
        else:
            self.get_logger().info("CONNECTION REFUSED!: {} REASON: {}".format(response, msg.data.split("_")[1]))

        
    def ResponsePing(self, msg):
        self.PublishStrMsg("ping_response", "robot_{}".format(self.name, self.connected), False)

    # def Spawn(self, canvas, pos):
    #     size = self.Coord2Size(pos)
    #     self.robot = canvas.create_rectangle(size[0], size[1], size[2], size[3], fill="red")

    
    # def Coord2Size(self, pos):
    #     x_ = pos[0] + self.size
    #     y_ = pos[1] + self.size
    
    #     return np.array([pos[0], pos[1], x_, y_])
    
    
    # def MoveForward(self, msg):
    #     self.pos = np.add(self.pos, self.velocity)
    #     msg = "{}_{}-{}".format(self.name, self.pos[0], self.pos[1])
    #     self.PublishStrMsg("robot_upd", msg)
    #     # print(self.pos)
        
    # def MoveBackward(self, msg):
    #     self.pos = np.subtract(self.pos, self.velocity)
    #     msg = "{}_{}-{}".format(self.name, self.pos[0], self.pos[1])
    #     self.PublishStrMsg("robot_upd", msg)
    #     # print(self.pos)
        

def main():
    rclpy.init()

    robot = Robot([1,0])
    rclpy.spin(robot)
    
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
