

import random
import time
from E160_environment import *
from E160_graphics import *

def main():  
       
    # instantiate robot navigation classes
    environment = E160_environment()
    graphics = E160_graphics(environment)
    
    # set time step size in seconds
    deltaT = 0.1
    # loop over time
    while True:
        # update graphics, but stop the thread if user stopped the gui
        if not graphics.update():
            break
        
        # update robots
        environment.update_robots(deltaT)
        
        # log all the robot data
        environment.log_data()
    
        # maintain timing
        time.sleep(deltaT)
            
main()
