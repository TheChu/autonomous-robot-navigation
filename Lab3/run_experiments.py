import math
import random
import time
from E160_environment import *

def forwards_experiments():
    # set time step size in seconds
    deltaT = 0.1

    points_file_name = 'Log/forwards_points.txt'
    data_file_name = 'Log/forwards_data.txt'

    for i in range(20):

        # instantiate robot navigation classes
        environment = E160_environment(data_file_name)
        environment.control_mode = "AUTONOMOUS CONTROL MODE"
        environment.track_mode = "POINT MODE"
        r = environment.robots[0]
        r.make_headers(points_file_name)

        x_des = float(i+1) / 10
        y_des = float(0)
        theta_des = float(0)
        r.state_des.set_state(x_des, y_des, theta_des)
        r.point_tracked = False

        # loop over time
        while not r.point_tracked:

            # update robots
            environment.update_robots(deltaT)

            # log all the robot data
            environment.log_data()

            # maintain timing
            time.sleep(deltaT)

        print r.state_est.x
        r.log_data(points_file_name)
        del environment

def rotation_experiments():
    # set time step size in seconds
    deltaT = 0.1

    points_file_name = 'Log/rotation_points.txt'
    data_file_name = 'Log/rotation_data.txt'

    for i in range(24):

        # instantiate robot navigation classes
        environment = E160_environment(data_file_name)
        environment.control_mode = "AUTONOMOUS CONTROL MODE"
        environment.track_mode = "POINT MODE"
        r = environment.robots[0]
        r.make_headers(points_file_name)

        x_des = float(0)
        y_des = float(0)
        theta_des = float((i + 1) * math.pi / 12)
        theta_des = r.angle_wrap(theta_des)
        r.state_des.set_state(x_des, y_des, theta_des)
        r.point_tracked = False

        # loop over time
        while not r.point_tracked:

            # update robots
            environment.update_robots(deltaT)

            # log all the robot data
            environment.log_data()

            # maintain timing
            time.sleep(deltaT)

        print r.state_est.theta
        r.log_data(points_file_name)
        del environment

def angled_experiments():
    # set time step size in seconds
    deltaT = 0.1

    points_file_name = 'Log/angled_points.txt'
    data_file_name = 'Log/angled_data.txt'

    for i in range(24):

        # instantiate robot navigation classes
        environment = E160_environment(data_file_name)
        environment.control_mode = "AUTONOMOUS CONTROL MODE"
        environment.track_mode = "POINT MODE"
        r = environment.robots[0]
        r.make_headers(points_file_name)

        angle = float(i * math.pi / 12)
        angle = r.angle_wrap(angle)
        d = float(0.25)

        x_des = d * math.cos(angle)
        y_des = d * math.sin(angle)
        theta_des = float(0)
        r.state_des.set_state(x_des, y_des, theta_des)
        r.point_tracked = False

        # loop over time
        while not r.point_tracked:

            # update robots
            environment.update_robots(deltaT)

            # log all the robot data
            environment.log_data()

            # maintain timing
            time.sleep(deltaT)

        print r.state_est.x, r.state_est.y, r.state_est.theta
        r.log_data(points_file_name)
        del environment

def main():
    # forwards_experiments()
    # rotation_experiments()
    angled_experiments()

main()
