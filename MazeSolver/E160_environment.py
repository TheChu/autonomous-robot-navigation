from E160_robot import *
from E160_state import *
from E160_wall import *
from E160_maze import *
import serial
import time
from xbee import XBee


class E160_environment:


    def __init__(self, maze = [], file_name = False):
        self.width = 2.0
        self.height = 1.2
        self.cell_length = 0.3

        self.maze = E160_maze(maze)

        # set up walls, putting top left point first
        self.walls = []
        for start, end, slope in self.maze.walls:
            self.walls.append(E160_wall([-0.9+ start[0] * self.cell_length,
                                         0.5 + start[1] * self.cell_length,
                                         -0.9 + end[0] * self.cell_length,
                                         0.5 + end[1] * self.cell_length], slope))

        # create vars for hardware vs simulation
        self.robot_mode = "SIMULATION MODE"#"SIMULATION MODE" or "HARDWARE MODE"
        self.control_mode = "AUTONOMOUS CONTROL MODE"
        self.track_mode = "PATH MODE"#"POINT MODE" or "PATH MODE"

        # setup xbee communication
        if (self.robot_mode == "HARDWARE MODE"):
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
            print" Setting up serial port"
            try:
                self.xbee = XBee(self.serial_port)
            except:
                print("Couldn't find the serial port")

        # Setup the robots
        self.num_robots = 1
        self.robots = []
        for i in range (0,self.num_robots):

            # TODO: assign different address to each bot
            r = E160_robot(self, '\x00\x0C', i, file_name)
            self.robots.append(r)

    def update_robots(self, deltaT):

        # loop over all robots and update their state
        for r in self.robots:

            # set the control actuation
            r.update(deltaT)


    def log_data(self):

        # loop over all robots and update their state
        for r in self.robots:
            r.log_data(r.file_name)

    def quit(self):
        self.xbee.halt()
        self.serial.close()
