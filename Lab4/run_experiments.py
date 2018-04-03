import math
import random
import time
from E160_environment import *

def angle_wrap(a):
    while a > math.pi:
        a = a - 2*math.pi
    while a < -math.pi:
        a = a + 2*math.pi

    return a

def random_experiments():
    # set time step size in seconds
    deltaT = 0.1

    points_file_name = 'Log/random_points2.txt'
    data_file_name = 'Log/random_data2.txt'

    for i in range(8):

        angle = float(i * math.pi / 4)
        angle = angle_wrap(angle)
        d = float(0.25)

        x_des = d * math.cos(angle)
        y_des = d * math.sin(angle)
        theta_des = float(0)

        for j in range(10):

            print i, j

            # instantiate robot navigation classes
            environment = E160_environment(False, data_file_name)
            environment.control_mode = "AUTONOMOUS CONTROL MODE"
            environment.track_mode = "POINT MODE"
            r = environment.robots[0]
            r.make_headers(points_file_name)

            r.state_des.set_state(x_des, y_des, theta_des)
            r.point_tracked = False

            failed = False

            # loop over time
            while not r.point_tracked:

                if r.totalT > 60:
                    failed = True
                    break

                # update robots
                environment.update_robots(deltaT)

                # log all the robot data
                environment.log_data()

                # maintain timing
                time.sleep(deltaT)

            print failed, r.state_est.x, r.state_est.y, r.state_est.theta
            r.log_data(points_file_name, failed)
            del environment

def known_experiments():
    # set time step size in seconds
    deltaT = 0.1

    points_file_name = 'Log/known_points2.txt'
    data_file_name = 'Log/known_data2.txt'

    for i in range(8):

        angle = float(i * math.pi / 4)
        angle = angle_wrap(angle)
        d = float(0.25)

        x_des = d * math.cos(angle)
        y_des = d * math.sin(angle)
        theta_des = float(0)

        for j in range(10):

            print i, j

            # instantiate robot navigation classes
            environment = E160_environment(True, data_file_name)
            environment.control_mode = "AUTONOMOUS CONTROL MODE"
            environment.track_mode = "POINT MODE"
            r = environment.robots[0]
            r.make_headers(points_file_name)

            r.state_des.set_state(x_des, y_des, theta_des)
            r.point_tracked = False

            failed = False

            # loop over time
            while not r.point_tracked:

                if r.totalT > 60:
                    failed = True
                    break

                # update robots
                environment.update_robots(deltaT)

                # log all the robot data
                environment.log_data()

                # maintain timing
                time.sleep(deltaT)

            print failed, r.state_est.x, r.state_est.y, r.state_est.theta
            r.log_data(points_file_name, failed)
            del environment

def main():
    random_experiments()
    known_experiments()

main()
