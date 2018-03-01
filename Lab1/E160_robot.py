
from E160_state import *
import math
import datetime

class E160_robot:

    def __init__(self, environment, address, robot_id):
        self.stopped = False
        self.environment = environment
        self.state = E160_state()
        self.state.set_state(0,0,0)
        self.state_des = E160_state()
        self.state_des.set_state(0,0,0)
        self.v = 0.05
        self.w = 0.1
        self.radius = 0.12
        self.wheel_radius = 0.03
        self.address = address
        self.ID = self.address.encode().__str__()[-1]
        self.last_measurements = []
        self.robot_id = robot_id
        self.manual_control_left_motor = 0
        self.manual_control_right_motor = 0
        self.file_name = 'Log/Bot' + str(self.robot_id) + '_' + datetime.datetime.now().replace(microsecond=0).__str__() + '.txt'
        self.range_measurements = []
        self.make_headers()

    def update(self, deltaT):

        # get sensor measurements
        encoder_measurements, range_measurements = self.update_sensor_measurements()

        self.range_measurements = range_measurements

        # localize
        self.localize(encoder_measurements, range_measurements, deltaT)

        # call motion planner
        #self.motion_planner.update_plan()

        # determine new control signals
        R, L = self.update_control(range_measurements)


        # send the control measurements to the robot
        self.send_control(R, L, deltaT)


    def update_sensor_measurements(self):

        # send to actual robot !!!!!!!!! John
        if self.environment.robot_mode == "HARDWARE MODE":
            command = '$S @'
            self.environment.xbee.tx(dest_addr = self.address, data = command)

            update = self.environment.xbee.wait_read_frame()

            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:]
            range_measurements = data[:-2]

            print "update sensors measurements ",encoder_measurements, range_measurements
            return encoder_measurements, range_measurements


        # obtain sensor measurements !!!!!! Chris
        elif self.environment.robot_mode == "SIMULATION MODE":
            return 1,1

        return 1,1


    def localize(self, encoder_measurements, range_measurements, deltaT):
        pass


    def update_control(self, range_measurements):

        if self.environment.control_mode == "MANUAL CONTROL MODE":
            R = self.manual_control_right_motor
            L = self.manual_control_left_motor

        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":

            speed = 45
            high_end = 580
            low_end  = 520

            L = 0
            R = 0

            # NEGATIVE = FORWARDS (our wheels are flipped)
            if range_measurements[0] > high_end:
              L= speed
              R=L*1.1

            # if range_measurements[0] > low_end and range_measurements[0] < high_end:
            #   self.stopped = True
            #   L = 0
            #   R = 0

            elif range_measurements[0] < low_end:
              L = -speed
              R = L*1.1


        return R, L

    def send_control(self, R, L, deltaT):

        # send to actual robot !!!!!!!!
        if self.environment.robot_mode == "HARDWARE MODE":
            if (L < 0):
                LDIR = 0
            else:
                LDIR = 1

            if (R < 0):
                RDIR = 0
            else:
                RDIR = 1
            RPWM = int(abs(R))
            LPWM = int(abs(L))

            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            self.environment.xbee.tx(dest_addr = self.address, data = command)


        # simulate kinematics
        elif self.environment.robot_mode == "SIMULATION MODE":
            self.state = self.update_state(deltaT, self.state, R, L)


    def update_state(self, deltaT, state, R, L):


        return state



    def make_headers(self):
        f = open(self.file_name, 'a+')
        f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} \n'.format('R1', 'R2', 'R3', 'RW', 'LW'))
        f.close()



    def log_data(self):
        f = open(self.file_name, 'a+')

        # edit this line to have data logging of the data you care about
        data = [str(x) for x in self.range_measurements]

        f.write(' '.join(data) + '\n')
        f.close()


    def set_manual_control_motors(self, R, L):

        if self.environment.robot_mode == "HARDWARE MODE":
            self.manual_control_right_motor = int(R*256/100)
            self.manual_control_left_motor = int(L*256/100)
        else:
            self.manual_control_right_motor = float(R)/500
            self.manual_control_left_motor = float(L)/500
