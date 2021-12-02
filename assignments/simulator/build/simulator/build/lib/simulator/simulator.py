import os
import sys
import json
import math
import rclpy
import threading
import tkinter as tk
import numpy as np

from tkinter import *
from tkinter import messagebox
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from PIL import Image, ImageTk

#TODO edit mode #FIXME map bug
#TODO walls
#FIXME close mainthread err

ROBOT_PNG = os.path.abspath(os.getcwd()) + "/src/simulator/resource/assets/robot.png"
FREE_CELL_COLOR = "#F0F0F0"
OBSTACLE_CELL_COLOR = "#313131"
BASE_COLOR = "#121212"

FONT_TITLE = ('Arial', 18, 'bold')
FONT_BASE = ('Arial', 14, 'normal')

class Simulator(Node, threading.Thread):
    def __init__(self, *args, **kwargs):
        super().__init__("Simulator")
        # tk.Tk.__init__(self, *args, **kwargs)
        threading.Thread.__init__(self)
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
        # Events
        self.ping_response = self.create_subscription(String, "world_ping", self.ResponsePing, 10)
        self.map_update_listener = self.create_subscription(String, "map_upd", self.UpdateMap, 10)
        self.robot_move_listener = self.create_subscription(Odometry, "current_pose", self.HandleMove, 10)
        self.goal_dist_listener = self.create_subscription(String, "current_distance", self.UpdateDistance ,10)
        
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
        
        ### Tkinter App config
        self.root = tk.Tk()
        self.root.title('ROS2 Navigation Simulator')
        self.root.geometry("1300x1000")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.CloseApp)
        self.root.configure(background=BASE_COLOR)
        self.canvas = tk.Canvas(self.root, 
                                width=self.map_size["w"],
                                height=self.map_size["h"],
                                bg=BASE_COLOR,
                                bd=0, highlightthickness=0)
        self.canvas.pack(side="top", fill="both", expand="true", padx=10, pady=10)
        ### Mouse events
        self.canvas.bind('<Button-1>', self.LeftClick)
        self.canvas.bind('<Button-2>', self.MiddleClick)
        self.canvas.bind('<Button-3>', self.RightClick)
        self.DrawMap()
        
        ### SIDEBAR
        # Text Vars
        self.robot_pose_from_gui = {"x": StringVar(), "y": StringVar(), "theta": StringVar()}
        self.goal_pose_str = {"x": StringVar(), "y": StringVar()}
        self.distance_var = StringVar()
        self.current = {"x": StringVar(), "y": StringVar(), "theta": StringVar(), "lin_vel": StringVar(), "ang_vel": StringVar()}
        self.current["x"].set("X: {:.2f}".format(self.robots[0]["odo"].pose.pose.position.x))
        self.current["y"].set("Y: {:.2f}".format(self.robots[0]["odo"].pose.pose.position.y))
        self.current["theta"].set(u"\u03b8: {:.2f}".format(self.robots[0]["odo"].pose.pose.orientation.z))
        self.current["lin_vel"].set("linear: {:.2f} px/s".format(self.robots[0]["odo"].twist.twist.linear.x))
        self.current["ang_vel"].set("angular: {:.2f} rad/s".format(self.robots[0]["odo"].twist.twist.angular.z))
        # Int Vars
        self.grid = tk.IntVar()
        self.grid.set(0)
        
        # widgets
        Label(self.root, text="Robot position", font=FONT_TITLE, anchor=CENTER, bg=BASE_COLOR, fg="#FFFFFF", padx=0, pady=10).place(x=1030, y=10)
        Label(self.root, textvariable=self.current["x"], font=FONT_BASE, bg=BASE_COLOR, fg="#FFFFFF", padx=15, pady=0).place(x=1020, y=60)
        Label(self.root, textvariable=self.current["y"], font=FONT_BASE, bg=BASE_COLOR, fg="#FFFFFF", padx=15, pady=0).place(x=1020, y=90)
        Label(self.root, textvariable=self.current["theta"], font=FONT_BASE, bg=BASE_COLOR, fg="#FFFFFF", padx=15, pady=0).place(x=1020, y=120)
        Label(self.root, textvariable=self.current["lin_vel"], font=FONT_BASE, bg=BASE_COLOR, fg="#FFFFFF", padx=15, pady=0).place(x=1020, y=150)
        Label(self.root, textvariable=self.current["ang_vel"], font=FONT_BASE, bg=BASE_COLOR, fg="#FFFFFF", padx=15, pady=0).place(x=1020, y=180)
        
        Label(self.root, text="Set position", font=FONT_TITLE, anchor=CENTER, bg=BASE_COLOR, fg="#FFFFFF", padx=0, pady=10).place(x=1030, y=210)
        Label(self.root, text="X:", font=FONT_BASE, bg=BASE_COLOR, fg="#FFFFFF", padx=0, pady=0).place(x=1030, y=260)
        Entry(self.root, textvariable=self.robot_pose_from_gui["x"], width=5).place(x=1060, y=260)
        Label(self.root, text="Y:", font=FONT_BASE, bg=BASE_COLOR, fg="#FFFFFF", padx=0, pady=0).place(x=1120, y=260)
        Entry(self.root, textvariable=self.robot_pose_from_gui["y"], width=5).place(x=1150, y=260)
        Label(self.root, text=u"\u03b8:", font=FONT_BASE, bg=BASE_COLOR, fg="#FFFFFF", padx=0, pady=0).place(x=1210, y=260)
        Entry(self.root, textvariable=self.robot_pose_from_gui["theta"], width=5).place(x=1240, y=260)
        Button(self.root, text="Respawn", command=self.RespawnRobot).place(x=1030, y=290)

        Label(self.root, text="Go to", font=FONT_TITLE, anchor=CENTER, bg=BASE_COLOR, fg="#FFFFFF", padx=0, pady=10).place(x=1030, y=340)
        Label(self.root, text="X: ", font=FONT_BASE, bg=BASE_COLOR, fg="#FFFFFF", padx=0, pady=0).place(x=1030, y=390)
        Entry(self.root, textvariable=self.goal_pose_str["x"], width=5).place(x=1060, y=390)
        Label(self.root, text="Y: ", font=FONT_BASE, bg=BASE_COLOR, fg="#FFFFFF", padx=0, pady=0).place(x=1120, y=390)
        Entry(self.root, textvariable=self.goal_pose_str["y"], width=5).place(x=1150, y=390)
        Button(self.root, text="Go", command=self.Move2Goal).place(x=1030, y=420)
        Label(self.root, textvariable=self.distance_var, font=FONT_BASE, bg=BASE_COLOR, fg="#FFFFFF", padx=15, pady=0).place(x=1020, y=460)
        
        Label(self.root, text="Map settings", font=FONT_TITLE, anchor=CENTER, bg=BASE_COLOR, fg="#FFFFFF", padx=0, pady=10).place(x=1030, y=510)
        self.grid_indicator = Checkbutton(self.root, text='Grid', variable=self.grid, onvalue=1, offvalue=0, font=FONT_BASE, bg=BASE_COLOR, selectcolor=BASE_COLOR, borderwidth=0, fg="#FFFFFF", padx=0, pady=0, command=self.Grid).place(x=1030, y=560)

        pub = self.create_publisher(String, "sim_connected", 10)
        msg = String()
        msg.data = "sim_ok"
        pub.publish(msg)
        self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(msg.data), msg.data))
        self.destroy_publisher(pub)
        self.init_timer.destroy()
        
        self.active_robot = self.robots[0]["name"]
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
            
            x = robot_data[1].split("-")[0]
            y = robot_data[1].split("-")[1]
            theta = 0.0
            self.SetPose(newborn, [x, y, theta])
            
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
        robot["odo"].pose.pose.orientation.z = float(pose[2])
    
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
    
    def DrawMap(self, ):
        self.GetMapMatrix()
        for x in range(len(self.map_matrix)):
            for y in range(len(self.map_matrix[x])):
                x1 = x * self.cell_size
                x2 = x1 + self.cell_size
                
                y1 = y * self.cell_size
                y2 = y1 + self.cell_size
                
                color = OBSTACLE_CELL_COLOR if self.map_matrix[x][y] == "1" else FREE_CELL_COLOR                   
                self.cells[x, y] = self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, tags="cell", outline=color)
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
        x  = math.floor(event.x / self.cell_size)
        y = math.floor(event.y / self.cell_size)
        print("clicked at x: {} y: {}".format(x, y))
        self.canvas.itemconfig(self.cells[(x, y)], fill=OBSTACLE_CELL_COLOR)        
        self.PublishStringMsg("map_update_coord", "{}_{}".format(x, y), False)

    def MiddleClick(self, event):
        #TODO bind mouse event        
        print("middle")

    def RightClick(self, event):
        x = math.floor(event.x / self.cell_size)
        y = math.floor(event.y / self.cell_size)
        print("clicked at x: {} y: {}".format(x, y))
        self.canvas.itemconfig(self.cells[(x, y)], fill=FREE_CELL_COLOR)        
        self.PublishStringMsg("map_update_coord", "{}_{}".format(x, y), False)       
    
    ### GUI Buttons Handlers
    def RespawnRobot(self): 
        self.canvas.itemconfig(self.cells[self.current_gaol[0], self.current_gaol[1]], fill=FREE_CELL_COLOR)        
        x = self.robot_pose_from_gui["x"].get()
        y = self.robot_pose_from_gui["y"].get()
        theta = self.robot_pose_from_gui["theta"].get()
        
        if len(x) == 0 or len(y) == 0 or len(theta) == 0:
            messagebox.showinfo("Position Error", "Please fill every input fields (x,y,theta)")
        else:
            self.SetPose(self.robots[0], [x,y,theta])
            start_x = self.robots[0]["odo"].pose.pose.position.x * self.cell_size
            start_y = self.robots[0]["odo"].pose.pose.position.y * self.cell_size
            # robot["item"] = self.canvas.create_rectangle(start_x, start_y, end_x, end_y, fill='red')
            # robot_ico_img = Image.open(ROBOT_PNG)
            # robot_ico_img = robot_ico_img.rotate(robot["odo"].pose.pose.orientation.z)
            self.robots[0]["icon"] = tk.PhotoImage(file=ROBOT_PNG)
            self.robots[0]["icon"] = self.RotateRobot(self.robots[0]["odo"].pose.pose.orientation.z)
            self.robots[0]["item"] = self.canvas.create_image(start_x, start_y, image=self.robots[0]["icon"])
            self.canvas.tag_raise(self.robots[0]["item"])
            self.canvas.update()
        
        # Update labels
        self.current["x"].set("X: {}".format(self.robots[0]["odo"].pose.pose.position.x))
        self.current["y"].set("Y: {}".format(self.robots[0]["odo"].pose.pose.position.y))
        self.current["theta"].set(u"\u03b8: {}".format(self.robots[0]["odo"].pose.pose.orientation.z))
        self.current["lin_vel"].set("linear: {:.2f} px/s".format(self.robots[0]["odo"].twist.twist.linear.x))
        self.current["ang_vel"].set("angular: {:.2f} rad/s".format(self.robots[0]["odo"].twist.twist.angular.z))
        self.robot_pose_from_gui["x"].set("")
        self.robot_pose_from_gui["y"].set("")
        self.robot_pose_from_gui["theta"].set("")
        # publish the new pose
        pub = self.create_publisher(Pose, "robot_respawn", 10)
        pub.publish(self.robots[0]["odo"].pose.pose)
        self.destroy_publisher(pub)
        
    def Move2Goal(self):
        try:
            self.canvas.itemconfig(self.cells[self.current_gaol[0], self.current_gaol[1]], fill=FREE_CELL_COLOR)
        except:
            pass      
        
        x = self.goal_pose_str["x"].get()
        y = self.goal_pose_str["y"].get()
        self.current_gaol = [int(x), int(y)]
        if len(x) == 0 or len(y) == 0:
            messagebox.showinfo("Goal Error", "Please fill every input fields (x,y)")
        else:
            self.canvas.itemconfig(self.cells[(int(x), int(y))], fill="red")        
            goal_pose = Pose()
            goal_pose.position.x = float(x)
            goal_pose.position.y = float(y)
            pub = self.create_publisher(Pose, "move_to_goal", 10)
            pub.publish(goal_pose)
            self.destroy_publisher(pub)
    
    def UpdateMap(self, msg):
        # self.get_logger().info("I got: {}".format(msg.data[0:25]))
        self.bit_map = msg.data
        # print(self.bit_map)
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
        start_x = msg.pose.pose.position.x * self.cell_size
        start_y = msg.pose.pose.position.y * self.cell_size
        self.robots[0]["icon"] = tk.PhotoImage(file=ROBOT_PNG)
        # TODO rotation
        self.robots[0]["icon"] = self.RotateRobot(-msg.pose.pose.orientation.z)
        self.robots[0]["item"] = self.canvas.create_image(start_x, start_y, image=self.robots[0]["icon"])
        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z]
        self.SetPose(self.robots[0], pose)
        self.current["x"].set("X: {:.2f}".format(self.robots[0]["odo"].pose.pose.position.x))
        self.current["y"].set("Y: {:.2f}".format(self.robots[0]["odo"].pose.pose.position.y))
        self.current["theta"].set(u"\u03b8: {:.2f}".format(self.robots[0]["odo"].pose.pose.orientation.z))
        self.current["lin_vel"].set("linear: {:.2f} px/s".format(msg.twist.twist.linear.x))
        self.current["ang_vel"].set("angular: {:.2f} rad/s".format(msg.twist.twist.angular.z))
        
        self.canvas.tag_raise(self.robots[0]["item"])
        self.canvas.update()
        # self.robots[0]["item"] = self.canvas.create_rectangle(start_x, start_y, end_x, end_y, fill='red')
    
    def UpdateDistance(self, msg):
        self.distance_var.set("Distance from goal: {:.2f} px".format(float(msg.data)))
    
    def Grid(self):
        color = OBSTACLE_CELL_COLOR if self.grid.get() == 1 else FREE_CELL_COLOR
        for cell in self.cells:
            self.canvas.itemconfig(self.cells[cell], outline=color)
        
def main():
    rclpy.init()
    app = Simulator()
    rclpy.spin_until_future_complete(app, app.init)
    rclpy.spin_until_future_complete(app, app.exit)
    
    # app.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()