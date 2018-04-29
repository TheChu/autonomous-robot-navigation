import random
import time
from E160_environment import *
from E160_graphics import *
from trackRobot import *

def main():

    # maze1 = [[[1,1,1,0], [1,0,0,0], [1,0,1,0], [1,0,0,1]],
    #          [[1,1,0,0], [0,0,0,0], [1,0,1,0], [0,0,1,1]],
    #          [[0,1,1,1], [0,1,1,0], [1,0,1,0], [1,0,0,1]]]
    print 0
    grid_arr, corners, bot_pos = getMaze()

    print grid_arr
    print corners
    print bot_pos
    while True:
        pass

    # instantiate robot navigation classes
    environment = E160_environment(grid_arr, corners, bot_pos)
    graphics = E160_graphics(environment)

    # set time step size in seconds
    deltaT = 0.1
    # loop over time
    while True:
        print '1'
        # update graphics, but stop the thread if user stopped the gui
        if not graphics.update():
            break

        frame = photoBot()                           # Get image from webcam
        color, thresh = filterFrame(frame, corners)   # Crops and thresholds image
        bot_pos = localizeBot(color, thresh)   # TODO
        print '2'

        # update robots
        environment.update_robots(deltaT, bot_pos)

        # log all the robot data
        environment.log_data()

        # maintain timing
        time.sleep(deltaT)

main()
