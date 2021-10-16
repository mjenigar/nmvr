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

class Simulator(Node, tk.Tk):
    def __init__(self, *args, **kwargs):
        super().__init__("simulator_listener")
        tk.Tk.__init__(self, *args, **kwargs)
        # self.robot = {} #TODO
        self.map_size = {}
        self.cell_size = None
        self.bit_map = None
        self.cells = {}
        
        ### Listeners
        self.listener_thread = threading.Thread(target=self.StartListening)        
        self.map_cfg_listener = self.create_subscription(String, "map_cfg", self.HandleMapCfg, 10)
        self.map_listener = self.create_subscription(String, "map", self.HandleMap,10)
        
        ### Tasks
        self.map_done = Future()   
        # self.map_done.add_done_callback()     
        
    ### LISTENER HANDLERS
    def StartListening(self):
        rclpy.spin(self)
    
    def HandleMapCfg(self, msg):
        self.get_logger().info("I got: {}".format(msg.data))
        s = msg.data.split("/")
        self.cell_size = int(s[1])
        self.map_size["w"] = int(s[0].split("x")[0])
        self.map_size["h"] = int(s[0].split("x")[1])
        
        self.canvas = tk.Canvas(self, 
                                width=self.map_size["w"],
                                height=self.map_size["h"],
                                borderwidth=0,
                                highlightthickness=0)
        self.canvas.pack(side="top", fill="both", expand="true")
        ### Mouse events
        self.canvas.bind('<Button-1>', self.LeftClick)
        self.canvas.bind('<Button-2>', self.MiddleClick)
        self.canvas.bind('<Button-3>', self.RightClick)               
        
        self.n_cells = int(self.map_size["w"]/self.cell_size)
        
    def HandleMap(self, msg):
        self.get_logger().info("I got: {}".format(msg.data[0:25]))
        self.bit_map = msg.data
        self.map_done.set_result(msg)
        self.listener_thread.start()
        self.DrawMap()
    
    # TODO FIXME
    def setRobot(self, rawStr):
        self.robot["x"] = rawStr.split("-")[0]
        self.robot["y"] = rawStr.split("-")[1]
    
    ### PUBLISHER HANDLERS
    def PublishMapUpdate(self):
        msg = String()
        self.pub = self.create_publisher(String, "map_upd", 10)
        msg.data = self.bit_map
        self.pub.publish(msg)
        self.get_logger().info("sent [{} Bytes] --> {}".format(sys.getsizeof(msg.data), msg.data[0:25] if len(msg.data) > 25 else msg.data))
        self.destroy_publisher(self.pub)
        
    def GetMapMatrix(self):
        x = 0
        self.map_matrix = [] 
        for x in range(self.n_cells):
            self.map_matrix.append(self.bit_map[x:x+self.n_cells])

        return self.map_matrix
    
    def Matrix2Bin(self):
        self.bit_map = ''.join(self.map_matrix)
    
    def DrawMap(self):
        self.GetMapMatrix()
        print(self.map_matrix)
        for x in range(len(self.map_matrix)):
            for y in range(len(self.map_matrix[x])):
                x1 = x * self.cell_size
                x2 = x1 + self.cell_size
                
                y1 = y * self.cell_size
                y2 = y1 + self.cell_size
                
                color = "black" if self.map_matrix[x][y] == "1" else "white"
                self.cells[x, y] = self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, tags="cell")
                
                #TODO handle robot
                # if self.robot["x"] == x and self.robot["y"] == y:
                #     self.canvas.create_oval(x1+2,y1+2,x2-2,y2-2, fill="blue", tags="robot")
        self.mainloop()
        
    def LeftClick(self, event):
        x = math.floor(event.x / self.cell_size)
        y = math.floor(event.y / self.cell_size)
        self.canvas.itemconfig(self.cells[(x, y)], fill="black")        
        tmp = list(self.map_matrix[x])
        tmp[y] = '1'
        self.map_matrix[x] = ''.join(tmp)
        self.Matrix2Bin()
        self.PublishMapUpdate()        

    def MiddleClick(self, event):
        #TODO bind mouse event        
        print("middle")

    def RightClick(self, event):
        #TODO bind mouse event        
        print("right")


if __name__ == "__main__":
    rclpy.init()
    l = Simulator()
    # l.listener_thread.start()
    rclpy.spin_until_future_complete(l, l.map_done)
    
    print("aaaaa")
    l.destroy_node()
    rclpy.shutdown()