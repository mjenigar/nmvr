import sys
import json
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

MAP_FILE = "map/map.json"

class World(Node):
    def __init__(self, file):
        super().__init__("world")
        self.world_config = self.ReadMap(file)
        self.world_map = self.world_config["map"]
        self.msg = String()
        
        ### Listeners
        self.world_listener = self.create_subscription(String, "map_upd", self.UpdateMap, 10)
        
    def ReadMap(self, file):
        with open(file) as f:
            config = json.load(f)
            
        return config
    
    def GetMapCfg(self):
        cfg_str = "{}x{}/{}".format(
            self.world_config["map_size"]["height"],
            self.world_config["map_size"]["width"],
            self.world_config["cell_size"]
        )
        
        return cfg_str
                
    def EncodeMap(self):
        code = ""
        for pos in self.world_map:
            code += str(pos["value"])
            
        return code
    
    def PublishStrMsg(self, topic, msg):
        self.pub = self.create_publisher(String, topic, 10)
        self.msg.data = msg
        self.pub.publish(self.msg)
        self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(self.msg.data), self.msg.data[0:25] if len(self.msg.data) > 25 else self.msg.data))
        self.destroy_publisher(self.pub)
        
    def UpdateMap(self, msg):
        self.get_logger().info("I got: {}".format(msg.data[0:25]))
        for i, cell in enumerate(self.world_map):
            cell["value"] = msg.data[i]
        self.world_config["map"] = self.world_map
        
        with open(MAP_FILE, 'w') as f:
            f.write(json.dumps(self.world_config, ensure_ascii=False, indent=4))        
        self.get_logger().info("Map updated")



