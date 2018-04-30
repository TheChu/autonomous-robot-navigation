from E160_robot import *
from E160_state import *
from E160_wall import *
from E160_maze import *
import serial
import time
from xbee import XBee


class E160_environment:


    def __init__(self, maze = [], corners = [], pixel_pos = (), file_name = False):
        self.width = 2.2
        self.height = 1.7
        self.cell_length = 0.4

        self.top_left_corner = corners[0]
        self.bottom_right_corner = corners[3]

        self.pixel_width = self.bottom_right_corner[0] - self.top_left_corner[0]
        self.pixel_height = self.bottom_right_corner[1] - self.top_left_corner[1]

        self.maze = E160_maze(maze)

        # set up walls, putting top left point first
        self.walls = []
        for start, end, slope in self.maze.walls:
            self.walls.append(E160_wall([(-self.width / 2 + 0.2) + start[0] * self.cell_length,
                                         (self.height / 2 - 0.2) + start[1] * self.cell_length,
                                         (-self.width / 2 + 0.2) + end[0] * self.cell_length,
                                         (self.height / 2 - 0.2) + end[1] * self.cell_length], slope))

        # create vars for hardware vs simulation
        self.robot_mode = "HARDWARE MODE"#"SIMULATION MODE" or "HARDWARE MODE"
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

            botActualPos = self.get_pos_from_pixel_pos(pixel_pos)

            # TODO: assign different address to each bot
            r = E160_robot(self, '\x00\x0C', i, botActualPos, file_name)
            self.robots.append(r)

    def get_pos_from_pixel_pos(self, pixel_pos):
        x = (-self.width / 2 + 0.2) + (pixel_pos[0] - self.top_left_corner[0]) * (self.width / self.pixel_width)
        y = (self.height / 2 - 0.2) - (pixel_pos[1] - self.top_left_corner[1]) * (self.height / self.pixel_height)
        theta = pixel_pos[2]
        return (x, y, theta)

    def update_robots(self, deltaT, bot_pos):

        # loop over all robots and update their state
        for r in self.robots:

            # set the control actuation
            r.update(deltaT, bot_pos)


    def log_data(self):

        # loop over all robots and update their state
        for r in self.robots:
            r.log_data(r.file_name)

    def quit(self):
        self.xbee.halt()
        self.serial.close()
