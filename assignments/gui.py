import sys

import json
import math
import tkinter as tk
import numpy as np

class Simulator(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        self.robot = {}
        self.map_size = {}
        self.cell_size = None
        self.bit_map = None
        
        self.cells = {}
        
        #TODO bind mouse events        
        # self.canvas.bind('<Button-1>', self.LeftClick)
        # self.canvas.bind('<Button-2>', self.MiddleClick)
        # self.canvas.bind('<Button-3>', self.RightClick)
    
    def setRobot(self, rawStr):
        self.robot["x"] = rawStr.split("-")[0]
        self.robot["y"] = rawStr.split("-")[1]
        
    def setMapSize(self, rawStr):
        s = rawStr.split("-")
        self.cell_size = s[1]
        self.map_size["w"] = s[0].split("x")[0]
        self.map_size["h"] = s[0].split("x")[1]
        
        self.canvas = tk.Canvas(self, 
                                width=self.map_size["w"],
                                height=self.map_size["h"],
                                borderwidth=0,
                                highlightthickness=0)
        self.canvas.pack(side="top", fill="both", expand="true")
        # FIXME error
        self.n_cells = int(self.map_size["w"]/self.cell_size)
        
    
    def GetMapMatrix(self):
        # TODO make the range dynamic 
        x = 0
        self.map_matrix = [] 
        for x in range(40):
            self.map_matrix.append(self.bit_map[x:x+40])
        
        return self.map_matrix
    
    def DrawMap(self):
        self.GetMapMatrix()
        for x in range(len(self.map_matrix)):
            for y in range(len(self.map_matrix[x])):
                x1 = x * self.cell_size
                x2 = x1 + self.cell_size
                
                y1 = y * self.cell_size
                y2 = y1 + self.cell_size
                
                color = "black" if self.map_matrix == "1" else "white"
                self.cells[x, y] = self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, tags="cell")

                if self.robot["x"] == x and self.robot["y"] == y:
                    self.canvas.create_oval(x1+2,y1+2,x2-2,y2-2, fill="blue", tags="robot")

    def LeftClick(self, event):
        x = math.floor(event.x / self.cell_size)
        y = math.floor(event.y / self.cell_size)
        self.canvas.itemconfig(self.backend_map[(x, y)], fill="red")

    def MiddleClick(self, event):
        print("middle")

    def RightClick(self, event):
        print("right")
    

import rclpy

from rclpy.node import Node
from std_msgs.msg import String

class GUI_listener(Node):
    def __init__(self):
        super().__init__("gui_listener")
        self.GUI = Simulator()
        self.subscription = self.create_subscription(String, "map", self.listener_callback, 10)
        self.subscription
        
    def listener_callback(self, msg):
        SUB = msg.data.split("_")[0]
        raw = msg.data.split("_")[1]
        
        if SUB == "robot":
            self.GUI.setRobot(raw)
        elif SUB == "mapconf":
            self.GUI.setMapSize(raw)
        elif SUB == "map":
            self.GUI.bit_map = raw

        self.get_logger().info("read {} [{} Bytes] --> {}".format(SUB, sys.getsizeof(msg.data), msg.data[0:25]))


    def LoadMap(self):
        print("Load Map")
        self.GUI.DrawMap()
        self.GUI.mainloop()
        
        # self.GUI.LoadMap()
        
    
    
# s = Simulator()
# s.DrawGrid()
# s.mainloop()

