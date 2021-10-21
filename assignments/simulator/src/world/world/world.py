import sys
import json
import rclpy

from rclpy.node import Node
from std_msgs.msg import String


class World(Node):
    def __init__(self, file):
        super().__init__("World")
        self.world_config = self.ReadMap(file)
        self.world_map = self.world_config["map"]
        self.msg = String()
        
        ### Listeners
        self.world_listener = self.create_subscription(String, "map_upd", self.UpdateMap, 10)
        self.robot_listener = self.create_subscription(String, "world_conn_req", self.ConnectRobot, 10)
        self.sim_listener = self.create_subscription(String, "sim_opened", self.HandleInitSimResponse, 10)
        self.move_forward_listener = self.create_subscription(String, "move_forward", self.HandleForward, 10)
        self.move_backward_listener = self.create_subscription(String, "move_forward", self.HandleBackward, 10)
        self.update_robot_pos_listener = self.create_subscription(String, "robot_upd", self.UpdateRobotPos, 10)
        
        ### Connected Robots pos
        self.available_robots = []
        
        ### Timers
        self.try_open_sim = self.create_timer(1, self.StartSimulator)
        
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
            print(pos)
            code += str(pos["value"])
            
        return code
    
    def GetRobotPos(self):
        _str = ""
        for i, robot in enumerate(self.available_robots):    
            _str += "{}_{}-{}".format(
                robot["name"], 
                robot["x"],
                robot["y"]
            )
            _str += "" if i == len(self.available_robots) - 1 else ","
        
        return _str
    
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

    def ConnectRobot(self, msg):
        data = msg.data.split("_")
        robot_name = data[0]
        pos = data[1].split("-")
        robot = {
            "name" : robot_name,
            "x": int(pos[0]),
            "y": int(pos[1])
        }
        
        self.available_robots.append(robot)        
        self.PublishStrMsg("world_connection", "true")
        
    def StartSimulator(self):
        if len(self.available_robots) > 0:
            self.PublishStrMsg("map_cfg", self.GetMapCfg())
            self.PublishStrMsg("map", self.EncodeMap())
            self.PublishStrMsg("robots", self.GetRobotPos())
        else:
            self.get_logger().info("Waiting for robot req ...")
    
    def HandleInitSimResponse(self, msg):
        self.try_open_sim.destroy()
        # self.sim_listener.destroy_subscription()
        self.get_logger().info("Simulator running ...")

    def HandleForward(self, msg):
        self.get_logger().info("{}".format(msg.data))
        robot2move = msg.data
        topic = "{}_forward".format(robot2move)
        self.PublishStrMsg(topic, "Get new pos")
    
    def HandleBackward(self, msg):
        self.get_logger().info("{}".format(msg.data))
        robot2move = msg.data
        topic = "{}_backward".format(robot2move)
        self.PublishStrMsg(topic, "Get new pos")

    def UpdateRobotPos(self, msg):
        self.get_logger().info("{}".format(msg.data))
        raw = msg.data.split("_")
        robot2upd = raw[0]
        for robot in self.available_robots:
            if robot["name"] == robot2upd:
                robot["x"] = float(raw[1].split("-")[0])
                robot["y"] = float(raw[1].split("-")[1])
                
        self.PublishStrMsg("robot_pos_update", msg.data)
        

import os

def main():
    dir_ = os.path.abspath(os.getcwd())
    MAP_FILE = dir_ + "/src/world/resource/map/map.json"
    
    print()

    rclpy.init()

    w = World(MAP_FILE)
    rclpy.spin(w)
    
    w.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
