
from E160_state import *
import math
import datetime

class E160_robot:

    def __init__(self, environment, address, robot_id):
        self.environment = environment
        self.state_est = E160_state()
        self.state_est.set_state(0,0,0)
        self.state_des = E160_state()
        self.state_des.set_state(0,0,0)
        #self.v = 0.05
        #self.w = 0.1
        self.R = 0
        self.L = 0
        self.radius = 0.147 / 2
        self.width = 2*self.radius
        self.wheel_radius = 0.03
        self.address = address
        self.ID = self.address.encode().__str__()[-1]
        self.last_measurements = []
        self.robot_id = robot_id
        self.manual_control_left_motor = 0
        self.manual_control_right_motor = 0
        self.file_name = 'Log/Bot' + str(self.robot_id) + '_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.txt'
        self.make_headers()
        self.encoder_resolution = 1440

        self.last_encoder_measurements = [0,0]
        self.encoder_measurements = [0,0]
        self.range_measurements = [0,0,0]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0
        self.rotated180 = False




    def update(self, deltaT):

        # get sensor measurements
        self.encoder_measurements, self.range_measurements = self.update_sensor_measurements(deltaT)

        # localize
        self.state_est = self.localize(self.state_est, self.encoder_measurements, self.range_measurements)

        # call motion planner
        #self.motion_planner.update_plan()

        # determine new control signals
        self.R, self.L = self.update_control(self.range_measurements)

        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)


    def update_sensor_measurements(self, deltaT):

        if self.environment.robot_mode == "HARDWARE MODE":
            command = '$S @'
            self.environment.xbee.tx(dest_addr = self.address, data = command)

            update = self.environment.xbee.wait_read_frame()

            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:]
            range_measurements = data[:-2]

        elif self.environment.robot_mode == "SIMULATION MODE":
            encoder_measurements = self.simulate_encoders(self.R, self.L, deltaT)
            range_measurements = [0,0,0]

        return encoder_measurements, range_measurements



    def localize(self, state_est, encoder_measurements, range_measurements):
        delta_s, delta_theta = self.update_odometry(encoder_measurements)
        state_est = self.update_state(state_est, delta_s, delta_theta)

        return state_est


    def update_control(self, range_measurements):

        if self.environment.control_mode == "MANUAL CONTROL MODE":
            R = self.manual_control_right_motor
            L = self.manual_control_left_motor

        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":
            if self.state_est.x >= 1.5:
              R = L = 0
            else:
              K_p = 75
              change = self.state_est.theta * K_p
              R = 80 - change
              L = 80 + change

            # t = 0
            # R = 50
            # L = -R
            # if self.state_est.theta < 0:
            #   self.rotated180 = True
            #   R = L = 0
            # if self.rotated180 and self.state_est.theta > t:
            #   R = L = 0

        return R, L

    def send_control(self, L, R, deltaT):

        # send to actual robot !!!!!!!!
        if self.environment.robot_mode == "HARDWARE MODE":
            if (L < 0):
                LDIR = 1
            else:
                LDIR = 0

            if (R < 0):
                RDIR = 1
            else:
                RDIR = 0
            RPWM = int(abs(R))
            LPWM = int(abs(L))

            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            # command = '$M ' + str(RDIR) + ' ' + str(RPWM) + ' ' + str(LDIR) + ' ' + str(LPWM) + '@'
            self.environment.xbee.tx(dest_addr = self.address, data = command)



    def simulate_encoders(self, R, L, deltaT):
        gain = 10
        right_encoder_measurement = -int(R*gain*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = -int(L*gain*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement

        print "simulate_encoders", R, L, right_encoder_measurement, left_encoder_measurement
        return [left_encoder_measurement, right_encoder_measurement]


    def make_headers(self):
        f = open(self.file_name, 'a+')
        f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} \n'.format('R1', 'R2', 'R3', 'RW', 'LW'))
        f.close()



    def log_data(self):
        f = open(self.file_name, 'a+')

        # edit this line to have data logging of the data you care about
        data = [str(x) for x in [1,2,3,4,5]]

        f.write(' '.join(data) + '\n')
        f.close()


    def set_manual_control_motors(self, R, L):

        self.manual_control_right_motor = int(R*256/100)
        self.manual_control_left_motor = int(L*256/100)



    def update_odometry(self, encoder_measurements):

        delta_s = 0
        delta_theta = 0

        # ****************** Additional Student Code: Start ************

        L = self.radius

        diffEncoder0 = encoder_measurements[0] - self.last_encoder_measurements[0]
        diffEncoder1 = encoder_measurements[1] - self.last_encoder_measurements[1]

        if diffEncoder0 > 1000 or diffEncoder1 > 1000:
          diffEncoder0 = 0
          diffEncoder1 = 0

        wheelDistR = self.wheel_radius * diffEncoder0 * (2 * math.pi / self.encoder_resolution)
        wheelDistL = self.wheel_radius * diffEncoder1 * (2 * math.pi / self.encoder_resolution)

        delta_s = (wheelDistR + wheelDistL) / 2
        delta_theta = (wheelDistR - wheelDistL) / (2 * L)

        self.last_encoder_measurements = encoder_measurements

        # ****************** Additional Student Code: End ************

        # keep this to return appropriate changes in distance, angle
        return delta_s, delta_theta




    def update_state(self, state, delta_s, delta_theta):

        # ****************** Additional Student Code: Start ************
        delta_x = delta_s * math.cos(state.theta + delta_theta / 2)
        delta_y = delta_s * math.sin(state.theta + delta_theta / 2)
        new_theta = (state.theta + delta_theta)

        state.x = state.x + delta_x
        state.y = state.y + delta_y
        state.theta = math.atan2(math.sin(new_theta), math.cos(new_theta))


        # ****************** Additional Student Code: End ************

        # keep this to return the updated state
        return state
