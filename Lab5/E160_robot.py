
from E160_state import *
from E160_PF import *
from E160_MP import *
import math
import datetime

class E160_robot:

    def __init__(self, environment, address, robot_id):
        self.environment = environment
        self.state_est = E160_state()
        self.state_est.set_state(0,0,0)
        self.state_des = E160_state()
        self.state_des.set_state(0,0,0)
        self.state_draw = E160_state()
        self.state_draw.set_state(0,0,0)
        self.state_odo = E160_state()
        self.state_odo.set_state(0,0,0) # real position for simulation
        self.state_error = E160_state()
        self.state_curr_dest = E160_state()
        self.state_curr_dest.set_state(0.0,0.0,0.0)

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

        self.Kpho = 2.0#1.0
        self.Kalpha = 3.0#2.0
        self.Kbeta = -1.0#-0.5
        self.Kp = 2.0
        self.max_velocity = 0.2
        self.point_tracked = True
        self.encoder_per_sec_to_rad_per_sec = 10
        self.epsilon = 0.01
        self.epsilon2 = 0.005
        self.error = 0.08
        self.min_rotation = 0.05
        self.max_rotation = 2
        self.min_ptrack_dist_error = 0.1
        self.min_ptrack_ang_error = math.pi
        self.max_ang_velocity = 0.5
        self.point_reached = False


        self.trajectory = []
        self.path_counter = 0

        # Path Planner and Particle Filter
        self.PF = E160_PF(environment, self.width, self.wheel_radius, self.encoder_resolution, True)
        self.MP = E160_MP(environment, self.state_odo, self.radius)
        self.build_path([0], self.MP.node_list)
        self.replan_path = False

    def update(self, deltaT):
        # get sensor measurements
        self.encoder_measurements, self.range_measurements = self.update_sensor_measurements(deltaT)

        # update odometry
        delta_s, delta_theta = self.update_odometry(self.encoder_measurements)

        # update simulated real position, find ground truth for simulation
        self.state_odo = self.localize(self.state_odo, delta_s, delta_theta, self.range_measurements)

        # localize with odometry
        self.state_est = self.state_odo

        # to out put the true location for display purposes only.
        self.state_draw = self.state_odo

        # call motion planner
        self.motion_plan()
        self.track_trajectory()

        # determine new control signals
        self.R, self.L = self.update_control(self.range_measurements)

        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)

        #print(self.state_est.x, self.state_est.y, self.state_est.t)
        # self.state_est.set_state(0,0,0)

    def motion_plan(self):
        if (self.replan_path == True):
            # Reset destination
            self.path_counter = 0
            self.state_curr_dest = self.state_est

            # Set goal node
            goal_node = E160_MP.Node(self.state_des.x, self.state_des.y)

            # Generate path with RRT
            node_indices = self.MP.update_plan(self.state_odo, goal_node)
            self.build_path(node_indices, self.MP.node_list)
            self.replan_path = False


        else:
            pass

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
            sensor1 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[0])
            sensor2 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[1])
            sensor3 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[2])
            range_measurements = [sensor1, sensor2, sensor3]

        return encoder_measurements, range_measurements



    def localize(self, state_est, delta_s, delta_theta, range_measurements):
        state_est = self.update_state(state_est, delta_s, delta_theta)

        return state_est



    def update_control(self, range_measurements):

        if self.environment.control_mode == "MANUAL CONTROL MODE":
            desiredWheelSpeedR = self.manual_control_right_motor
            desiredWheelSpeedL = self.manual_control_left_motor

        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":
            desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()

        return desiredWheelSpeedR, desiredWheelSpeedL

    def sign(self, x):
        if x == 0:
            return 0
        elif x > 0:
            return 1
        else:
            return -1

    def point_tracker_control(self):

        # If the desired point is not tracked yet, then track it
        if not self.point_tracked:


            ############ Student code goes here ############################################
            delta_x = self.state_curr_dest.x - self.state_est.x
            delta_y = self.state_curr_dest.y - self.state_est.y
            pho = (delta_x**2 + delta_y**2)**0.5

            if pho < self.epsilon:
                self.point_reached = True

            if self.point_reached:

                # self.point_tracked = True
                desiredWheelSpeedL = 0
                desiredWheelSpeedR = 0

                delta_theta = self.state_curr_dest.theta - self.state_est.theta
                delta_theta = self.angle_wrap(delta_theta)

                if math.fabs(delta_theta) < self.epsilon2:
                    self.point_tracked = True
                    self.point_reached = False
                    desiredWheelSpeedL = 0
                    desiredWheelSpeedR = 0

                else:
                    desiredW = self.Kp * delta_theta
                    if math.fabs(desiredW) < self.min_rotation:
                        desiredW = self.sign(desiredW) * self.min_rotation
                    elif math.fabs(desiredW) > self.max_rotation:
                        desiredW = self.sign(desiredW) * self.max_rotation
                    desiredWheelSpeedR = self.encoder_per_sec_to_rad_per_sec * desiredW * self.radius / self.wheel_radius
                    desiredWheelSpeedL = -self.encoder_per_sec_to_rad_per_sec * desiredW * self.radius / self.wheel_radius

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

                beta = -self.state_est.theta - alpha + self.state_curr_dest.theta
                beta = self.angle_wrap(beta)

                desiredV = self.Kpho * pho
                if goalBehind:
                    desiredV = -desiredV

                if math.fabs(desiredV) > self.max_velocity:
                    desiredV = self.sign(desiredV) * self.max_velocity

                desiredW = self.Kalpha * alpha + self.Kbeta * beta

                desiredWheelSpeedL = -self.encoder_per_sec_to_rad_per_sec * (desiredW * self.radius + desiredV) / self.wheel_radius
                desiredWheelSpeedR = -self.encoder_per_sec_to_rad_per_sec * (desiredV - desiredW * self.radius) / self.wheel_radius


        # the desired point has been tracked, so don't move
        else:
            desiredWheelSpeedR = 0
            desiredWheelSpeedL = 0

        return desiredWheelSpeedR,desiredWheelSpeedL

    def track_trajectory(self):
        '''Update the self.state_curr_dest for the point tracker  '''

        # calculate state error
        self.state_error = self.state_curr_dest-self.state_est
        error = self.state_error

        if (self.state_est.xydist(self.state_curr_dest) < self.min_ptrack_dist_error and abs(error.theta) < self.min_ptrack_ang_error):
            self.point_tracked = True
            self.state_curr_dest = self.trajectory[self.path_counter]
            if self.path_counter < len(self.trajectory) - 1:
                self.path_counter = self.path_counter+1
        else:
            self.point_tracked = False

    def send_control(self, L, R, deltaT):

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

    def simulate_encoders(self, R, L, deltaT):
        right_encoder_measurement = -int(R*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = -int(L*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement

        #print "simulate_encoders", R, L, right_encoder_measurement, left_encoder_measurement
        return [left_encoder_measurement, right_encoder_measurement]

    def simulate_range_finder(self, state, sensorT):
        '''Simulate range readings, given a simulated ground truth state'''
        p = self.PF.Particle(state.x, state.y, state.theta, 0)

        return self.PF.FindMinWallDistance(p, self.environment.walls, sensorT)

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

        # Add Code: copy your odometry code here
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

        # keep this to return appropriate changes in distance, angle
        return delta_s, delta_theta

    def update_state(self, state, delta_s, delta_theta):

        # Add Code: copy your state update code here
        delta_x = delta_s * math.cos(state.theta + delta_theta / 2)
        delta_y = delta_s * math.sin(state.theta + delta_theta / 2)
        new_theta = (state.theta + delta_theta)

        state.x = state.x + delta_x
        state.y = state.y + delta_y
        state.theta = math.atan2(math.sin(new_theta), math.cos(new_theta))


        # keep this to return the updated state
        return state

    def build_path(self, node_indices, node_list):
        '''Update the trajectory using the node indices return by the path
        planner'''
        self.trajectory = []
        prev_node = node_list[0]
        self.trajectory.append(E160_state(prev_node.x, prev_node.y, 0))
        for index in node_indices[1:]:
            current_node = node_list[index]
            prev_node = node_list[index-1]
            desired_angle = -self.angle_wrap(math.atan2(current_node.y -prev_node.y,
                current_node.x - prev_node.x))
            desired_state = E160_state(current_node.x, current_node.y, 0)
            self.trajectory.append(desired_state)

    def angle_wrap(self, a):
        while a > math.pi:
            a = a - 2*math.pi
        while a < -math.pi:
            a = a + 2*math.pi
        return a

    def ang_diff(self, a, b):
        return self.angle_wrap(a - b)
