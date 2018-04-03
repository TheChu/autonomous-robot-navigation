import math
import random
import time
from E160_environment import *

def random_experiments():
    # set time step size in seconds
    deltaT = 0.1

    points_file_name = 'Log/random_points2.txt'
    data_file_name = 'Log/random_data2.txt'

    for i in range(100):

        print i

        # instantiate robot navigation classes
        environment = E160_environment(False, data_file_name)
        environment.control_mode = "AUTONOMOUS CONTROL MODE"
        environment.track_mode = "POINT MODE"
        r = environment.robots[0]
        r.make_headers(points_file_name)

        # x and y are random floats from -0.5 to 0.5
        x_des = random.random() - 0.45
        y_des = random.random() - 0.45
        # theta is a random float from -pi to pi
        theta_des = random.random() * 2 * math.pi
        theta_des = r.angle_wrap(theta_des)
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

    for i in range(100):

        print i

        # instantiate robot navigation classes
        environment = E160_environment(True, data_file_name)
        environment.control_mode = "AUTONOMOUS CONTROL MODE"
        environment.track_mode = "POINT MODE"
        r = environment.robots[0]
        r.make_headers(points_file_name)

        # x and y are random floats from -0.5 to 0.5
        x_des = random.random() - 0.45
        y_des = random.random() - 0.45
        # theta is a random float from -pi to pi
        theta_des = random.random() * 2 * math.pi
        theta_des = r.angle_wrap(theta_des)
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
    # forwards_experiments()
    # rotation_experiments()
    # angled_experiments()
    random_experiments()
    known_experiments()

main()
