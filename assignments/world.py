import sys
import json
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

class World():
    # first argument - path to map.json file 
    def __init__(self, file):
        self.world_config = self.ReadMap(file)
        self.world_map = self.world_config["map"]
        
        self.PUB = True

    def ReadMap(self, file):
        with open(file) as f:
            config = json.load(f)
            
        return config
        
    def EncodeMap(self):
        code = ""
        for pos in self.world_map:
            code += str(pos["value"])
            
        return code


class pub_World(Node):
    # first argument - path to map.json file 
    def __init__(self):
        super().__init__("world")
        self.world = World("map/map.json")
        self.publisher = self.create_publisher(String, "map", 10)    
        self.msg = String()
        
        self.PUB = "robot"
        self.timer = self.create_timer(1, self.callback)
        
    def callback(self):
        if self.PUB == "robot":
            data = self.world.world_config["robot"]
    
            self.msg.data = "{}_{}-{}".format(self.PUB, data["start_x"], data["start_y"])
            self.PUB = 'mapconf'
        elif self.PUB == "mapconf":
            h = self.world.world_config["map_size"]["height"]
            w = self.world.world_config["map_size"]["width"]
            cell_size = self.world.world_config["cell_size"]
    
            self.msg.data = "{}_{}x{}-{}".format(self.PUB, h, w, cell_size)            
            self.PUB = "map"
        elif self.PUB == "map":
            self.msg.data = "{}_{}".format(self.PUB, self.world.EncodeMap())

        self.publisher.publish(self.msg)
        self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(self.msg.data), self.msg.data[0:25]))