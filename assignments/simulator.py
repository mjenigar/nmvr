import json
import math
import tkinter as tk
import numpy as np

class Simulator(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        with open('map.json') as f:
            map_json = json.load(f)

        self.robot = map_json["robot"]
        self.map_h = map_json["map_size"]["height"]
        self.map_w = map_json["map_size"]['width']        
        self.cell_size = map_json["cell_size"]
        self.map = map_json["map"]
        self.n_cells = int(self.map_h/self.cell_size)
        self.canvas = tk.Canvas(self, 
                                width=self.map_w,
                                height=self.map_h,
                                borderwidth=0,
                                highlightthickness=0)
        self.canvas.pack(side="top", fill="both", expand="true")
        self.backend_map = {}
        
        self.canvas.bind('<Button-1>', self.LeftClick)
        self.canvas.bind('<Button-2>', self.MiddleClick)
        self.canvas.bind('<Button-3>', self.RightClick)
        
    def DrawGrid(self):
        for cell in self.map:
            x1 = cell["x"] * self.cell_size
            x2 = x1 + self.cell_size
            
            y1 = cell["y"] * self.cell_size
            y2 = y1 + self.cell_size

            color = "black" if cell["value"] == 1 else "white"
            self.backend_map[cell["x"], cell["y"]] = self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, tags="cell")

            if self.robot["start_x"] == cell["x"] and self.robot["start_y"] == cell["y"]:
                self.canvas.create_oval(x1+2,y1+2,x2-2,y2-2, fill="blue", tags="robot")

    def LeftClick(self, event):
        x = math.floor(event.x / self.cell_size)
        y = math.floor(event.y / self.cell_size)
        self.canvas.itemconfig(self.backend_map[(x, y)], fill="red")

    def MiddleClick(self, event):
        print("middle")

    def RightClick(self, event):
        print("right")
    
    
s = Simulator()
s.DrawGrid()
s.mainloop()

