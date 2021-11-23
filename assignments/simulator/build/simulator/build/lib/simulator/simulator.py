import sys
import json
import math
import threading
import tkinter as tk
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String

#TODO create logs
#TODO create edit mode
#TODO https://stackoverflow.com/questions/17466561/best-way-to-structure-a-tkinter-application


class Simulator(Node, threading.Thread):
    def __init__(self, *args, **kwargs):
        super().__init__("Simulator")
        # tk.Tk.__init__(self, *args, **kwargs)
        threading.Thread.__init__(self)

        ######
        ### Map
        self.map_size = {}
        self.cell_size = None
        self.bit_map = None
        self.cells = {}
        ######

        ######
        ### Status
        self.status = "Searching for connection"        
        self.robots = []
        self.active_robot = None
        self.change = True       
        ######
        
                
        ### Flags
        self.map_cfg_flag = False
        self.map_flag = False
        self.robot_flag = False
        
        ### Listeners
        # Init
        self.map_cfg_listener = self.create_subscription(String, "map_cfg", self.HandleMapCfg, 10)
        self.map_listener = self.create_subscription(String, "map", self.HandleMap, 10)
        self.robot_pos_listener = self.create_subscription(String, "robots", self.HandleRobots, 10)
        
        # Events
        self.ping_response = self.create_subscription(String, "world_ping", self.ResponsePing, 10)

        self.map_update_listener = self.create_subscription(String, "map_upd", self.UpdateMap, 10)
        self.robot_pos_update_listener = self.create_subscription(String, "robot_pos_update", self.UpdatePosition, 10)
        
        ### Tasks
        self.init = Future()   
        self.exit = Future()
        
        ### Timers
        self.init_timer = self.create_timer(1, self.InitSimulator)
        self.status_timer = self.create_timer(1, self.GetStatus)

        ### TMP
        self.velocity = np.array([1, 0])
    
    def GetStatus(self):
        if self.change:
            if len(self.robots) > 0:
                robots = ""
                for r in self.robots:
                    robots += r["name"] + " "
            else:
                robots = "None"
            
            self.get_logger().info("STATUS: {} ROBOTS: {} ACTIVE: {}".format(self.status, robots, self.active_robot))
            self.change = False
    
    def ResponsePing(self, msg):
        self.PublishStringMsg("ping_response", "simulator_ok")
    
    def CloseApp(self):
        self.root.destroy()
        self.exit.set_result("exit")
        self.destroy_node()
    
    def run(self):
        self.get_logger().info("Starting simulator ...")
        self.init.set_result("OK")
        self.init_timer.destroy()
        
        self.root = tk.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.CloseApp)
        
        self.canvas = tk.Canvas(self.root, 
                                width=self.map_size["w"],
                                height=self.map_size["h"],
                                borderwidth=0,
                                highlightthickness=0)
        self.canvas.pack(side="top", fill="both", expand="true")
        
        ### Mouse events
        self.canvas.bind('<Button-1>', self.LeftClick)
        self.canvas.bind('<Button-2>', self.MiddleClick)
        self.canvas.bind('<Button-3>', self.RightClick)
        
        ### Controls events
        self.root.bind('<Left>', self.Left)           
        self.root.bind('<Right>', self.Right)           
        self.root.bind('<Up>', self.Up)           
        self.root.bind('<Down>', self.Down)     

        self.DrawMap()
        
        pub = self.create_publisher(String, "sim_connected", 10)
        msg = String()
        msg.data = "sim_ok"
        pub.publish(msg)
        self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(msg.data), msg.data))
        self.destroy_publisher(pub)
        self.init_timer.destroy()
        
        self.active_robot = self.robots[0]
        self.root.mainloop()        

    def InitSimulator(self):
        if self.map_cfg_flag and self.map_flag and self.robot_flag:
            self.start()
        # else:
        #     self.get_logger().info("Waiting for data ...")
    
    def HandleMapCfg(self, msg):
        s = msg.data.split("/")
        self.cell_size = int(s[1])
        self.map_size["w"] = int(s[0].split("x")[0])
        self.map_size["h"] = int(s[0].split("x")[1])
        
        self.n_cells = int(self.map_size["w"]/self.cell_size)
        self.map_cfg_flag = True
        if not self.map_cfg_flag:
            self.get_logger().info("I got: {}".format(msg.data))
            self.get_logger().info("Map Config - OK")
        
    def HandleMap(self, msg):
        self.bit_map = msg.data
        self.map_flag = True
        if not self.map_flag:
            self.get_logger().info("I got: {}".format(msg.data[0:25]))
            self.get_logger().info("Map - OK")

    def HandleRobots(self, msg):
        avail_robots = msg.data.split(",")
        for robot in avail_robots:
            robot_data = robot.split("_")
            newborn = {
                "name": robot_data[0],
                "size": 12,
                "x": int(robot_data[1].split("-")[0]),
                "y": int(robot_data[1].split("-")[1]),
                "item" : None
            }
            
            if len(self.robots) > 0:
                for other in self.robots:
                    if other["name"] != newborn["name"]:
                        self.robots.append(newborn)           
                    else:
                        return
            else:
                self.robots.append(newborn)
                
        if len(self.robots) > 0:
            self.robot_flag = True
        
        if not self.robot_flag:            
            self.get_logger().info("I got: {}".format(msg))
            self.get_logger().info("Robots - OK")

        
    ### PUBLISHER HANDLERS
    def PublishStringMsg(self, topic, msg_, log=True):
        msg = String()
        pub = self.create_publisher(String, topic, 10)
        msg.data = msg_
        pub.publish(msg)
        if log:
            self.get_logger().info("{} [{} Bytes] --> {}".format(topic, sys.getsizeof(msg.data), msg.data[0:25] if len(msg.data) > 25 else msg.data))
        self.destroy_publisher(pub)
    
    # def PublishMapUpdate(self):
    #     msg = String()
    #     self.pub = self.create_publisher(String, "map_upd", 10)
    #     msg.data = self.bit_map
    #     self.pub.publish(msg)
    #     self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(msg.data), msg.data[0:25] if len(msg.data) > 25 else msg.data))
    #     self.destroy_publisher(self.pub)
        
    def GetMapMatrix(self):
        x = 0
        self.map_matrix = [] 
        for x in range(self.n_cells):
            self.map_matrix.append(self.bit_map[x:x+self.n_cells])

        return self.map_matrix
    
    def Matrix2Bin(self):
        self.bit_map = ""
        for x in range(self.n_cells):
            for y in range(self.n_cells):
                self.bit_map += self.map_matrix[x][y]
        
        # self.bit_map = ''.join(self.map_matrix)
    
    def DrawMap(self):
        self.GetMapMatrix()
        for x in range(len(self.map_matrix)):
            for y in range(len(self.map_matrix[x])):
                x1 = x * self.cell_size
                x2 = x1 + self.cell_size
                
                y1 = y * self.cell_size
                y2 = y1 + self.cell_size
                
                color = "black" if self.map_matrix[x][y] == "1" else "white"
                self.cells[x, y] = self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, tags="cell")
                self.canvas.tag_lower(self.cells[x, y])
                for robot in self.robots:
                    if robot["x"] == x and robot["y"] == y:
                        start_x = robot["x"] * self.cell_size
                        end_x = start_x + robot["size"]
                        
                        start_y = robot["y"] * self.cell_size
                        end_y = start_y + robot["size"]
                        
                        robot["item"] = self.canvas.create_rectangle(start_x, start_y, end_x, end_y, fill='red')
                        self.canvas.tag_raise(robot["item"])
    
    ### EVENT CONTROLS
    def LeftClick(self, event):
        x = math.floor(event.x / self.cell_size)
        y = math.floor(event.y / self.cell_size)
        print("clicked at x: {} y: {}".format(x, y))
        # self.canvas.itemconfig(self.cells[(x, y)], fill="black")        
        self.PublishStringMsg("map_update_coord", "{}_{}".format(x, y))
        # tmp = list(self.map_matrix[x])
        # tmp[y] = '1'
        
        # self.map_matrix[x] = ''.join(tmp)
        # print(self.map_matrix)
        # self.Matrix2Bin()
        # self.PublishMapUpdate()        
    
    def MiddleClick(self, event):
        #TODO bind mouse event        
        print("middle")

    def RightClick(self, event):
        #TODO bind mouse event        
        print("right")
        
    def Right(self, event):
        # self.PublishStringMsg("move_forward", self.active_robot["name"])
        self.canvas.move(self.robots[0]["item"], self.robots[0]["x"] +  self.velocity[0], self.robots[0]["y"] + self.velocity[1])
        
    def Left(self, event):
        # self.PublishStringMsg("move_backward", self.active_robot["name"])
        self.canvas.move(self.robots[0]["item"], self.robots[0]["x"] -  self.velocity[0], self.robots[0]["y"] - self.velocity[1])

    def Up(self, event):
        # TODO rotation up
        new_vel = [self.velocity[0], self.velocity[1] - 0.1]
        self.velocity = np.array(new_vel)
        print("Rotating UP Current {}".format(self.velocity[1]))
        
    def Down(self, event):
        # TODO rotation down
        new_vel = [self.velocity[0], self.velocity[1] + 0.1]
        self.velocity = np.array(new_vel)
        print("Rotating DOWN Current {}".format(self.velocity[1]))

    ### Move handlers
    def UpdatePosition(self, msg):
        self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(msg.data), msg.data[0:25] if len(msg.data) > 25 else msg.data))
        raw = msg.data.split("_")
        robot2upd = raw[0]
        for robot in self.robots:
            if robot["name"] == robot2upd:
                robot["x"] = float(raw[1].split("-")[0])
                robot["y"] = float(raw[1].split("-")[1])

                self.canvas.move(robot["item"], robot["x"], robot["y"])

    def UpdateMap(self, msg):
        self.get_logger().info("I got: {}".format(msg.data[0:25]))
        self.bit_map = msg.data
        print(self.bit_map)
        # self.canvas.itemconfig(self.cells[(x, y)], fill="black")        

        self.get_logger().info("Map updated")
        
    
def main():
    rclpy.init()
    app = Simulator()
    rclpy.spin_until_future_complete(app, app.init)
    rclpy.spin_until_future_complete(app, app.exit)
    
    # app.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()