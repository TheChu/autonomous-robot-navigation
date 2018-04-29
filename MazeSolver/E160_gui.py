import random
import time
from E160_environment import *
from E160_graphics import *
from trackRobot import *

def main():

    # maze1 = [[[1,1,1,0], [1,0,0,0], [1,0,1,0], [1,0,0,1]],
    #          [[1,1,0,0], [0,0,0,0], [1,0,1,0], [0,0,1,1]],
    #          [[0,1,1,1], [0,1,1,0], [1,0,1,0], [1,0,0,1]]]

    grid_arr, corners, bot_spots = getMaze()
    pixelPos = getPixelPos(bot_spots)

    # instantiate robot navigation classes
    environment = E160_environment(grid_arr, corners, pixelPos)
    graphics = E160_graphics(environment)

    # set time step size in seconds
    deltaT = 0.1
    # loop over time
    while True:
        # update graphics, but stop the thread if user stopped the gui
        if not graphics.update():
            break

        frame = photoBot()                           # Get image from webcam
        color, thresh = filterFrame(frame, corners)   # Crops and thresholds image
        pixelPos = localizeBot(color, thresh)
        botPos = environment.get_pos_from_pixel_pos(pixelPos)

        # update robots
        environment.update_robots(deltaT, botPos)

        # log all the robot data
        environment.log_data()

        # maintain timing
        time.sleep(deltaT)

main()
