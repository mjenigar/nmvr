import os
import sys
import json
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

dir_ = os.path.abspath(os.getcwd())
MAP_FILE = dir_ + "/src/world/resource/map/map.json"

class World(Node):
    def __init__(self, file):
        super().__init__("World")
        self.world_config = self.ReadMap(file)
        self.world_map = self.world_config["map"]
        self.msg = String()
        ### Status
        self.status = "wait"
        self.available_robots = []
        self.simulator = "disconnected"
        self.change = True
        self.WorldStatus = self.create_timer(1, self.GetStatus)
        ### Ping
        self.robots_responds = []
        self.connected_simulator = 0 
        ### pub & sub
        self.ping = self.create_timer(3, self.PingAll)
        self.Search4Robot = self.create_subscription(String, "connection_req", self.ConnectRobot, 10)
        self.sim_listener = self.create_subscription(String, "sim_connected", self.HandleInitSimResponse, 10)
        self.robots_status = self.create_subscription(String, "ping_response", self.PingResponse, 10)
        self.map_listener = self.create_subscription(String, "map_update_coord", self.UpdateMap, 10)
        ### Timers
        self.try_open_sim = self.create_timer(1, self.StartSimulator)
    
    def ConnectRobot(self, msg):
        data = msg.data.split("_")
        robot_name = data[0]
        pos = data[1].split("-")
        robot = {
            "name" : robot_name,
            "x": float(pos[0]),
            "y": float(pos[1])
        }
        
        if robot_name in [robot_["name"] for robot_ in self.available_robots]:
            response = "false_Robot with this name is already connected...".format(robot_name)        
        else:
            response = "true"
            self.available_robots.append(robot)        
            self.change = True
            self.get_logger().info("{} connected".format(robot_name))

        self.PublishStrMsg("world_connection", response)
        
    def GetStatus(self):
        if self.change:
            self.get_logger().info("STATUS: {} ROBOTS: {} SIMULATOR: {}".format(self.status, len(self.available_robots), self.simulator))
            self.change = False
    
    def PingAll(self):
        if len(self.available_robots) > 0 and self.simulator != "disconnected":
            self.PublishStrMsg("world_ping", "HEY!", False)
            self.wait4resp = self.create_timer(1, self.Check4PingResponse)
    
    def Check4PingResponse(self):
        if len(self.robots_responds) != len(self.available_robots):
            for robot in self.available_robots:
                delete = True
                for _robot in self.robots_responds:
                    if(robot["name"] == _robot):
                        delete = False
                        break

                if delete:
                    self.available_robots.remove(robot)
                    self.get_logger().info("{} disconnected".format(robot["name"]) )
                    self.change = True
                    self.delete = False
                    
        if self.connected_simulator != 1:
            self.simulator = "disconnected"
            self.try_open_sim = self.create_timer(1, self.StartSimulator)
            self.change = True

        self.robots_responds = []
        self.connected_simulator = 0
        self.wait4resp.destroy()
    
    def PingResponse(self, msg):
        resp_from = msg.data.split("_")[0]
        if resp_from == "robot":
            name = msg.data.split("_")[1]
            # self.get_logger().info("{} alive".format(name))
            self.robots_responds.append(name)
        elif resp_from == "simulator":
            self.connected_simulator += 1
                
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
        
    def UpdateMap(self, msg):
        self.get_logger().info("I got: {}".format(msg.data))
        print(msg.data.split("_"))
        x = msg.data.split("_")[0]
        y = msg.data.split("_")[1]
        
        for i in range(len(self.world_map)):
            if self.world_map[i]["x"] == int(x) and self.world_map[i]["y"] == int(y):
                self.world_map[i]["value"] = 1 if self.world_map[i]["value"] == 0 else 0 
                break            
        
        self.world_config["map"] = self.world_map
        
        with open(MAP_FILE, 'w') as f:
            f.write(json.dumps(self.world_config, ensure_ascii=False, indent=4))        
        self.get_logger().info("Map updated")
        self.PublishStrMsg("map_upd", self.EncodeMap())
        
    def StartSimulator(self):
        if len(self.available_robots) > 0:
            self.PublishStrMsg("map_cfg", self.GetMapCfg(), False)
            self.PublishStrMsg("map", self.EncodeMap(), False)
            self.PublishStrMsg("robots", self.GetRobotPos(), False)
        
    def HandleInitSimResponse(self, msg):
        self.try_open_sim.destroy()
        # self.destroy_subscription(self.sim_listener)
        self.simulator = "connected"
        self.status = "idle"
        self.change = True

    def PublishStrMsg(self, topic, msg, log=True):
        self.pub = self.create_publisher(String, topic, 10)
        self.msg.data = msg
        self.pub.publish(self.msg)
        if log:
            self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(self.msg.data), self.msg.data[0:25] if len(self.msg.data) > 25 else self.msg.data))
        self.destroy_publisher(self.pub)

def main():
    rclpy.init()
    w = World(MAP_FILE)
    rclpy.spin(w)
    w.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
