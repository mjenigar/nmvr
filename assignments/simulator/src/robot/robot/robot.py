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
        self.world = None
        self.pos = np.array(pos)
        self.velocity = np.array([0.01, 0])
        
        self.size = 12
        
        ### Flags
        self.connected = False
        
        ### Listeners
        self.world_conn_listener = self.create_subscription(String, "world_connection", self.Connect2World, 10)
        self.forward_listener = self.create_subscription(String, "{}_forward".format(self.name), self.MoveForward, 10)
        self.backward_listener = self.create_subscription(String, "{}_backward".format(self.name), self.MoveBackward, 10)

        ### Timers
        self.timer = self.create_timer(1, self.Search4World)

    def PublishStrMsg(self, topic, _msg, f=10):
        pub = self.create_publisher(String, topic, f)
        msg = String()
        msg.data = _msg
        pub.publish(msg)
        self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(msg.data), msg.data))
        self.destroy_publisher(pub)
    
    def Search4World(self):
        self.conn_req = self.create_publisher(String, "world_conn_req", 10)
        msg = String()
        msg.data = "{}_{}-{}".format(self.name, self.pos[0], self.pos[1])
        self.conn_req.publish(msg)
        self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(msg.data), msg.data))
        # self.destroy_publisher(self.conn_req)
    
    def Connect2World(self, msg):
        self.connected = True
        self.timer.destroy()
        
        print("Connected...")

    def Spawn(self, canvas, pos):
        size = self.Coord2Size(pos)
        self.robot = canvas.create_rectangle(size[0], size[1], size[2], size[3], fill="red")

    def Update(self):
        self.robot.delete("all")
        self.robot = self.world.create_rectangle(event.x + 5,event.y + 5, position)
    
    def Coord2Size(self, pos):
        x_ = pos[0] + self.size
        y_ = pos[1] + self.size

        return np.array([pos[0], pos[1], x_, y_])
    
    def MoveForward(self, msg):
        self.pos = np.add(self.pos, self.velocity)
        msg = "{}_{}-{}".format(self.name, self.pos[0], self.pos[1])
        self.PublishStrMsg("robot_upd", msg)
        # print(self.pos)
        
    def MoveBackward(self, msg):
        self.pos = np.subtract(self.pos, self.velocity)
        msg = "{}_{}-{}".format(self.name, self.pos[0], self.pos[1])
        self.PublishStrMsg("robot_upd", msg)
        # print(self.pos)
        

def main():
    rclpy.init()

    isaac = Robot([0,0])
    rclpy.spin(isaac)
    
    isaac.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
