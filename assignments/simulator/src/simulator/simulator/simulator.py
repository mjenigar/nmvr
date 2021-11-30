import os
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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from PIL import Image, ImageTk

#TODO create logs
#TODO create edit mode

ROBOT_PNG = os.path.abspath(os.getcwd()) + "/src/simulator/resource/assets/robot.png"

class Simulator(Node, threading.Thread):
    def __init__(self, *args, **kwargs):
        super().__init__("Simulator")
        # tk.Tk.__init__(self, *args, **kwargs)
        threading.Thread.__init__(self)
        self.__init = False
        ### Map
        self.map_size = {}
        self.cell_size = None
        self.bit_map = None
        self.cells = {}
        ### Status
        self.status = "Searching for connection"        
        self.robots = []
        self.active_robot = None
        self.change = True       
        ### Flags
        self.map_cfg_flag = False
        self.map_flag = False
        self.robot_flag = False
        ### Listeners
        # Init
        self.map_cfg_listener = self.create_subscription(String, "map_cfg", self.HandleMapCfg, 10)
        self.map_listener = self.create_subscription(String, "map", self.HandleMap, 10)
        self.robot_listener = self.create_subscription(String, "robots", self.HandleRobots, 10)
        self.robot_move_listener = self.create_subscription(Odometry, "current_pose", self.HandleMove, 10)
        # Events
        self.ping_response = self.create_subscription(String, "world_ping", self.ResponsePing, 10)
        self.map_update_listener = self.create_subscription(String, "map_upd", self.UpdateMap, 10)
        
        ### Tasks
        self.init = Future()   
        self.exit = Future()
        
        ### Timers
        self.init_timer = self.create_timer(1, self.InitSimulator)
        self.status_timer = self.create_timer(1, self.GetStatus)

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
        self.PublishStringMsg("ping_response", "simulator_ok", False)
    
    def CloseApp(self):
        self.root.destroy()
        self.exit.set_result("exit")
        self.destroy_node()
        rclpy.shutdown()
    
    def run(self):
        self.get_logger().info("Starting simulator ...")
        self.init.set_result("OK")
        self.init_timer.destroy()
        self.status = "idle"
        self.change = True
        
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
        
        self.DrawMap()
        
        pub = self.create_publisher(String, "sim_connected", 10)
        msg = String()
        msg.data = "sim_ok"
        pub.publish(msg)
        self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(msg.data), msg.data))
        self.destroy_publisher(pub)
        self.init_timer.destroy()
        
        self.active_robot = self.robots[0]["name"]
        self.__init = True
        self.root.mainloop()        

    def InitSimulator(self):
        if self.map_cfg_flag and self.map_flag and self.robot_flag:
            self.start()
    
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
                "size": 32,
                "wheel_r": 1.0,
                "odo" : Odometry(),
                "icon": None,
                "item" : None
            }
            self.SetPose(newborn, robot_data[1].split("-"))
            
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

    def SetPose(self, robot, pose):
        robot["odo"].pose.pose.position.x = float(pose[0])
        robot["odo"].pose.pose.position.y = float(pose[1])
    
    ### PUBLISHER HANDLERS
    def PublishStringMsg(self, topic, msg_, log=True):
        msg = String()
        pub = self.create_publisher(String, topic, 10)
        msg.data = msg_
        pub.publish(msg)
        if log:
            self.get_logger().info("{} [{} Bytes] --> {}".format(topic, sys.getsizeof(msg.data), msg.data[0:25] if len(msg.data) > 25 else msg.data))
        self.destroy_publisher(pub)
    
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
                    if robot["odo"].pose.pose.position.x == x and robot["odo"].pose.pose.position.y == y:
                        start_x = robot["odo"].pose.pose.position.x * self.cell_size
                        start_y = robot["odo"].pose.pose.position.y * self.cell_size
                        # robot["item"] = self.canvas.create_rectangle(start_x, start_y, end_x, end_y, fill='red')
                        # robot_ico_img = Image.open(ROBOT_PNG)
                        # robot_ico_img = robot_ico_img.rotate(robot["odo"].pose.pose.orientation.z)
                        robot["icon"] = tk.PhotoImage(file=ROBOT_PNG)
                        robot["icon"] = self.RotateRobot(robot["odo"].pose.pose.orientation.z)
                        robot["item"] = self.canvas.create_image(start_x, start_y, image=robot["icon"])
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
        
    def UpdateMap(self, msg):
        self.get_logger().info("I got: {}".format(msg.data[0:25]))
        self.bit_map = msg.data
        print(self.bit_map)
        # self.canvas.itemconfig(self.cells[(x, y)], fill="black")        

        self.get_logger().info("Map updated")

    def RotateRobot(self, angle):
        angleInRads = angle
        diag = math.sqrt(self.robots[0]["icon"].width()**2 + self.robots[0]["icon"].height()**2)
        x_midpoint = self.robots[0]["icon"].width()/2
        y_midpoint = self.robots[0]["icon"].height()/2
        rotated = tk.PhotoImage(width=int(diag), height=int(diag))
        for x in range(self.robots[0]["icon"].width()):
            for y in range(self.robots[0]["icon"].height()):
                # convert to ordinary mathematical coordinates
                x_new = float(x)
                y_new = float(-y)
                # shift to origin
                x_new -= x_midpoint
                y_new += y_midpoint
                # new rotated variables, rotated around origin (0,0) using simoultaneous assigment
                x_new, y_new = x_new * math.cos(angleInRads) - y_new * math.sin(angleInRads), x_new * math.sin(angleInRads) + y_new * math.cos(angleInRads)
                # shift back to quadrant iv (x,-y), but centered in bigger box
                x_new += diag/2
                y_new -= diag/2
                # convert to -y coordinates
                x_new = x_new
                y_new = -y_new
                # get pixel data from the pixel being rotated in hex format
                rgb = '#%02x%02x%02x' % self.robots[0]["icon"].get(x, y)
                # put that pixel data into the new image
                rotated.put(rgb, (int(x_new), int(y_new)))
                # this helps fill in empty pixels due to rounding issues
                rotated.put(rgb, (int(x_new+1), int(y_new)))

        return rotated
    
    def HandleMove(self, msg):
        if self.__init:
            # self.canvas.delete(self.robots[0]["item"])
            start_x = msg.pose.pose.position.x * self.cell_size
            start_y = msg.pose.pose.position.y * self.cell_size
            
            delta_x = msg.pose.pose.position.x - self.robots[0]["odo"].pose.pose.position.x
            delta_y = msg.pose.pose.position.y - self.robots[0]["odo"].pose.pose.position.y
            
            self.robots[0]["icon"] = tk.PhotoImage(file=ROBOT_PNG)
            # self.robots[0]["icon"] = self.RotateRobot(-msg.pose.pose.orientation.z)
            self.robots[0]["item"] = self.canvas.create_image(start_x, start_y, image=self.robots[0]["icon"])
            self.canvas.tag_raise(self.robots[0]["item"])
            self.canvas.update()

            # robot["item"] = self.canvas.create_image(start_x, start_y, image=tk_img)
            # self.robots[0]["item"] = self.canvas.create_rectangle(start_x, start_y, end_x, end_y, fill='red')
    
def main():
    rclpy.init()
    app = Simulator()
    rclpy.spin_until_future_complete(app, app.init)
    rclpy.spin_until_future_complete(app, app.exit)
    
    # app.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()