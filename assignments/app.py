import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from world import *
from gui import *

import _thread as thread


def run():
    rclpy.init()
    
    if sys.argv[1] == "pub":
        world = pub_World()
        # rclpy.spin_once(robot)
        
        rclpy.spin(world)
        
        world.destroy_node()
        rclpy.shutdown()
    elif sys.argv[1] == "sub":
        gui = GUI_listener()
        try:
            thread.start_new_thread(gui.LoadMap, ())
        except:
            print("Error: unable to start thread")
        rclpy.spin(gui)

        
        gui.destroy_node()
        rclpy.shutdown()
    else:
        print("wrong argv")


if __name__ == "__main__":
    run()