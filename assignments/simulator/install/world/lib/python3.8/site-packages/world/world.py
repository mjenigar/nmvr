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

        ######
        ### Status
        self.status = "wait"
        self.available_robots = []
        self.simulator = "disconnected"
        self.change = True

        self.WorldStatus = self.create_timer(1, self.GetStatus)
        ######
        
        ######
        ### Ping
        self.robots_responds = []
        self.connected_simulator = 0 
        # pub
        self.ping = self.create_timer(10, self.PingAll)
        # sub
        self.robots_status = self.create_subscription(String, "ping_response", self.PingResponse, 10)
        ######
        
        
        ### Listeners
        # Robot
        # self.disconnect_listen = self.create_subscription(String, "disconnect", self.DisconnectRobot, 10)
        # Init
        self.Search4Robot = self.create_subscription(String, "connection_req", self.ConnectRobot, 10)
        self.sim_listener = self.create_subscription(String, "sim_opened", self.HandleInitSimResponse, 10)

        # StatusCheck
        
        # Events
        self.map_listener = self.create_subscription(String, "map_update_coord", self.UpdateMap, 10)
        # self.move_forward_listener = self.create_subscription(String, "move_forward", self.HandleForward, 10)
        # self.move_backward_listener = self.create_subscription(String, "move_forward", self.HandleBackward, 10)
        # self.update_robot_pos_listener = self.create_subscription(String, "robot_upd", self.UpdateRobotPos, 10)
        
        ### Timers
        # self.try_open_sim = self.create_timer(1, self.StartSimulator)
    
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
        self.change = True
        
        self.get_logger().info("{} connected".format(robot_name))

    def GetStatus(self):
        if self.change:
            self.get_logger().info("STATUS: {} ROBOTS: {} SIMULATOR: {}".format(self.status, len(self.available_robots), self.simulator))
            self.change = False
    
    def isSimulatorOpened(self):
        return True if (self.simulator != None) else False
    
    def PingAll(self):
        if len(self.available_robots) > 0 or self.simulator != "disconnected":
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
    
    # def DisconnectRobot(self, msg):
    #     robot = msg.data.split("_")[1]
    #     for robot_ in self.available_robots:
    #         if robot == robot_["name"]:
    #             self.available_robots.remove(robot)
    #             self.get_logger().info("{} disconnected".format(robot))
    #             self.change = True

    
    
    
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
    
    def PublishStrMsg(self, topic, msg, log=True):
        self.pub = self.create_publisher(String, topic, 10)
        self.msg.data = msg
        self.pub.publish(self.msg)
        if log:
            self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(self.msg.data), self.msg.data[0:25] if len(self.msg.data) > 25 else self.msg.data))
        self.destroy_publisher(self.pub)
        
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
            self.PublishStrMsg("map_cfg", self.GetMapCfg())
            self.PublishStrMsg("map", self.EncodeMap())
            self.PublishStrMsg("robots", self.GetRobotPos())
        # else:
        #     self.get_logger().info("Waiting for robot to join ...")
    
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
        

def main():
    rclpy.init()

    w = World(MAP_FILE)
    rclpy.spin(w)
    
    w.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
