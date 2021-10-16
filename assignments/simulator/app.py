import rclpy

from world import World
# from simulator import Simulator


MAP_FILE = "map/map.json"

def main():
    rclpy.init()

    w = World(MAP_FILE)
    w.PublishStrMsg("map_cfg", w.GetMapCfg())
    w.PublishStrMsg("map", w.EncodeMap())
    rclpy.spin(w)
    
    w.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()    
    
