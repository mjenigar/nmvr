import numpy as np
import json

class MapGenerator():
    def __init__(self, robot_start_pos, map_size, cell_size):
        self.robot_start_pos = robot_start_pos
        self.map_size = map_size
        self.cell_size = cell_size
        
        self.json = {}
        
    def GenerateMap(self):
        self.json["robot"] = {"start_x" : self.robot_start_pos[0], "start_y" : self.robot_start_pos[1]}
        self.json["map_size"] = {"height" : self.map_size, "width": self.map_size}
        self.json["cell_size"] = self.cell_size
        
        n_cells = int(self.map_size/self.cell_size)
        
        cells = []
        for i in range(n_cells):
            for j in range(n_cells):
                val = 1 if np.random.rand() > .5 else 0
                cells.append({"x" : i, "y": j, "value": val})
        
        self.json["map"] = cells
        
        with open('map.json', 'w') as f:
            f.write(json.dumps(self.json, ensure_ascii=False, indent=4))
        
        

m = MapGenerator((0,0), 1000, 25)
m.GenerateMap()