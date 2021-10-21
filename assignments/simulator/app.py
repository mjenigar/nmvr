import rclpy

from world import World
from robot import Robot


# from simulator import Simulator


MAP_FILE = "map/map.json"

def main():
    rclpy.init()

    w = World(MAP_FILE)
    rclpy.spin(w)
    
    w.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()    
    
