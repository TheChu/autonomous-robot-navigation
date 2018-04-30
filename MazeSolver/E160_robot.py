from E160_state import *
import math
import datetime
import os.path

class E160_robot:

    def __init__(self, environment, address, robot_id, bot_pos = (), file_name = False):
        self.environment = environment
        self.state_est = E160_state()
        self.state_des = E160_state()
        if bot_pos:
            self.state_est.set_state(*bot_pos)
            self.state_des.set_state(*bot_pos)
        else:
            self.state_est.set_state(0,0,0)
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
        if file_name == False:
            file_name = 'Log/Bot' + str(self.robot_id) + '_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.txt'
        self.file_name = file_name
        self.make_headers(self.file_name)
        self.encoder_resolution = 1440

        self.last_encoder_measurements = [0,0]
        self.encoder_measurements = [0,0]
        self.range_measurements = [0,0,0]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0

        self.Kpho = 2.0#1.0
        self.Kalpha = 3.0#2.0
        self.Kbeta = -1.0#-0.5
        self.Kp = 2.0
        self.max_velocity = 0.05
        self.max_wheel_speed = 50
        self.min_wheel_speed = 20
        self.point_tracked = True
        self.point_reached = False
        self.encoder_per_sec_to_rad_per_sec = 10
        self.epsilon = 0.1
        self.epsilon2 = 0.1
        self.error = 0.08
        self.min_rotation = 0.05
        self.max_rotation = 2

        start = (0,0,1)
        goal = (4,3,2)
        instructions = self.environment.maze.aStarSearch(start, goal)
        self.instructions = self.translate_instructions(instructions)
        print self.instructions


    def update(self, deltaT, bot_pos):

        # get sensor measurements
        self.encoder_measurements, self.range_measurements = self.update_sensor_measurements(deltaT)

        # localize
        # self.state_est = self.localize(self.state_est, self.encoder_measurements, self.range_measurements)
        self.state_est.set_state(*bot_pos)

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

    def angle_wrap(self, a):
        while a > math.pi:
            a = a - 2*math.pi
        while a < -math.pi:
            a = a + 2*math.pi

        return a

    def update_control(self, range_measurements):

        if self.environment.control_mode == "MANUAL CONTROL MODE":
            desiredWheelSpeedR = self.manual_control_right_motor
            desiredWheelSpeedL = self.manual_control_left_motor

        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":
            if self.environment.track_mode == "POINT MODE":
                desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()
            elif self.environment.track_mode == "PATH MODE":
                desiredWheelSpeedR, desiredWheelSpeedL = self.path_tracker_control()
            else:
                desiredWheelSpeedR, desiredWheelSpeedL = 0

        return desiredWheelSpeedR, desiredWheelSpeedL

    def translate_instructions(self, instructions):
        transInstr = []
        currTheta = self.state_des.theta
        for direction in instructions:
            if currTheta != direction:
                transInstr.append((False, direction))
                currTheta = direction
            transInstr.append((True, direction))
        return transInstr

    def sign(self, x):
        if x == 0:
            return 0
        elif x > 0:
            return 1
        else:
            return -1

    def path_tracker_control(self):

        if self.point_tracked and self.instructions:
            isOriented, direction = self.instructions.pop(0)
            if isOriented:
                theta_des = self.state_des.theta
                x_des = self.state_des.x + self.environment.cell_length * math.cos(theta_des)
                y_des = self.state_des.y + self.environment.cell_length * math.sin(theta_des)
            else:
                theta_des = direction
                x_des = self.state_des.x
                y_des = self.state_des.y
            self.state_des.set_state(x_des, y_des, theta_des)
            print self.state_des.x, self.state_des.y, self.state_des.theta
            self.point_tracked = False

        return self.point_tracker_control()


    def point_tracker_control(self):

        # If the desired point is not tracked yet, then track it
        if not self.point_tracked:


            ############ Student code goes here ############################################
            delta_x = self.state_des.x - self.state_est.x
            delta_y = self.state_des.y - self.state_est.y
            pho = (delta_x**2 + delta_y**2)**0.5

            if pho < self.epsilon:
                self.point_reached = True

            if self.point_reached:

                delta_theta = self.state_des.theta - self.state_est.theta
                delta_theta = self.angle_wrap(delta_theta)

                if math.fabs(delta_theta) < self.epsilon2:
                    self.point_tracked = True
                    self.point_reached = False
                    desiredWheelSpeedL = 0
                    desiredWheelSpeedR = 0

                else:
                    desiredW = self.Kp * delta_theta
                    # if math.fabs(desiredW) < self.min_rotation:
                    #     desiredW = self.sign(desiredW) * self.min_rotation
                    # elif math.fabs(desiredW) > self.max_rotation:
                    #     desiredW = self.sign(desiredW) * self.max_rotation
                    desiredWheelSpeedL = self.encoder_per_sec_to_rad_per_sec * desiredW * self.radius / self.wheel_radius
                    desiredWheelSpeedR = -self.encoder_per_sec_to_rad_per_sec * desiredW * self.radius / self.wheel_radius

                    if desiredW != 0:
                        # Prevent robot from not moving
                        ratio = desiredWheelSpeedL / desiredWheelSpeedR
                        if math.fabs(desiredWheelSpeedL) < math.fabs(desiredWheelSpeedR):
                            desiredWheelSpeedL = self.sign(desiredWheelSpeedL) * self.min_wheel_speed
                            desiredWheelSpeedR = desiredWheelSpeedL / ratio
                        else:
                            desiredWheelSpeedR = self.sign(desiredWheelSpeedR) * self.min_wheel_speed
                            desiredWheelSpeedL = desiredWheelSpeedR * ratio
            else:

                alpha = -self.state_est.theta + math.atan2(delta_y, delta_x)
                alpha = self.angle_wrap(alpha)

                if math.fabs(alpha) > (math.pi / 2 + self.error):
                    goalBehind = True
                else:
                    goalBehind = False

                if goalBehind:
                    alpha = -self.state_est.theta + math.atan2(-delta_y, -delta_x)
                    alpha = self.angle_wrap(alpha)

                beta = -self.state_est.theta - alpha + self.state_des.theta
                beta = self.angle_wrap(beta)

                desiredV = self.Kpho * pho
                if goalBehind:
                    desiredV = -desiredV
                #
                # if math.fabs(desiredV) > self.max_velocity:
                #     desiredV = self.sign(desiredV) * self.max_velocity

                desiredW = self.Kalpha * alpha + self.Kbeta * beta

                # convertRatio = (self.encoder_resolution / (2 * math.pi * self.encoder_per_sec_to_rad_per_sec))
                desiredWheelSpeedL = self.encoder_per_sec_to_rad_per_sec * (desiredW * self.radius + desiredV) / self.wheel_radius
                desiredWheelSpeedR = self.encoder_per_sec_to_rad_per_sec * (desiredV - desiredW * self.radius) / self.wheel_radius

                if desiredV != 0:
                    # maxWheelSpeed = self.encoder_per_sec_to_rad_per_sec * self.max_velocity / self.wheel_radius
                    # Prevent robot from not moving
                    if (math.fabs(desiredWheelSpeedL) < self.min_wheel_speed) or (math.fabs(desiredWheelSpeedR) < self.min_wheel_speed):
                        ratio = desiredWheelSpeedL / desiredWheelSpeedR
                        if math.fabs(desiredWheelSpeedL) < math.fabs(desiredWheelSpeedR):
                            desiredWheelSpeedL = self.sign(desiredWheelSpeedL) * self.min_wheel_speed
                            desiredWheelSpeedR = desiredWheelSpeedL / ratio
                        else:
                            desiredWheelSpeedR = self.sign(desiredWheelSpeedR) * self.min_wheel_speed
                            desiredWheelSpeedL = desiredWheelSpeedR * ratio
                    # Prevent robot from moving too fast
                    elif (math.fabs(desiredWheelSpeedL) > self.max_wheel_speed) or (math.fabs(desiredWheelSpeedR) > self.max_wheel_speed):
                        ratio = desiredWheelSpeedL / desiredWheelSpeedR
                        if math.fabs(desiredWheelSpeedL) > math.fabs(desiredWheelSpeedR):
                            desiredWheelSpeedL = self.sign(desiredWheelSpeedL) * self.max_wheel_speed
                            desiredWheelSpeedR = desiredWheelSpeedL / ratio
                        else:
                            desiredWheelSpeedR = self.sign(desiredWheelSpeedR) * self.max_wheel_speed
                            desiredWheelSpeedL = desiredWheelSpeedR * ratio

        # the desired point has been tracked, so don't move
        else:
            desiredWheelSpeedR = 0
            desiredWheelSpeedL = 0

        return desiredWheelSpeedR,desiredWheelSpeedL


    def send_control(self, L, R, deltaT):

        # send to actual robot !!!!!!!!
        if self.environment.robot_mode == "HARDWARE MODE":

            L *= -1
            R *= -1

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

            command = '$M ' + str(RDIR) + ' ' + str(RPWM) + ' ' + str(LDIR) + ' ' + str(LPWM) + '@'
            self.environment.xbee.tx(dest_addr = self.address, data = command)



    def simulate_encoders(self, R, L, deltaT):
        right_encoder_measurement = int(R*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = int(L*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement

        # print "simulate_encoders", R, L, right_encoder_measurement, left_encoder_measurement
        return [left_encoder_measurement, right_encoder_measurement]


    def make_headers(self, file_name):
        if not os.path.isfile(file_name):
            f = open(file_name, 'a+')
            f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} {5:^1} \n'.format('X', 'Y', 'Theta', 'DesX', 'DesY', 'DesTheta'))
            f.close()



    def log_data(self, file_name):
        f = open(file_name, 'a+')

        # edit this line to have data logging of the data you care about
        data = [str(x) for x in [self.state_est.x,
                                 self.state_est.y,
                                 self.state_est.theta,
                                 self.state_des.x,
                                 self.state_des.y,
                                 self.state_des.theta
                                ]]

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
