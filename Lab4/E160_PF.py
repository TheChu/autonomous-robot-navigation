import math
import random
import numpy as np
import copy
from E160_state import*
from scipy.stats import norm


class E160_PF:

	def __init__(self, environment, robotWidth, wheel_radius, encoder_resolution):
		self.particles = []
		self.environment = environment
		self.numParticles = 400

		# maybe should just pass in a robot class?
		self.robotWidth = robotWidth
		self.radius = robotWidth/2
		self.wheel_radius = wheel_radius
		self.encoder_resolution = encoder_resolution
		self.FAR_READING = 1000

		# PF parameters
		self.IR_sigma = 0.2 # Range finder s.d
		self.odom_xy_sigma = 1.25	# odometry delta_s s.d
		self.odom_heading_sigma = 0.75	# odometry heading s.d
		self.particle_weight_sum = 0

		# define the sensor orientations
		self.sensor_orientation = [-math.pi/2, 0, math.pi/2] # orientations of the sensors on robot
		self.walls = self.environment.walls

		# initialize the current state
		self.state = E160_state()
		self.state.set_state(0,0,0)

		# TODO: change this later
		self.map_maxX = 1.0
		self.map_minX = -1.0
		self.map_maxY = 1.0
		self.map_minY = -1.0
		self.InitializeParticles()
		self.last_encoder_measurements = [0,0]

	def InitializeParticles(self):
		''' Populate self.particles with random Particle
			Args:
				None
			Return:
				None'''
		self.particles = [None for _ in range(self.numParticles)]
		for i in range(self.numParticles):
			# self.SetRandomStartPos(i)
			self.SetKnownStartPos(i)


	def SetRandomStartPos(self, i):
		x = self.map_minX + random.random() * (self.map_maxX - self.map_minX)
		y = self.map_minY + random.random() * (self.map_maxY - self.map_minY)
		heading = -math.pi + random.random() * (2 * math.pi)
		weight = 1.0 / self.numParticles
		self.particles[i] = self.Particle(x, y, heading, weight)

	def SetKnownStartPos(self, i):
		x = 0.0
		y = 0.0
		heading = 0.0
		weight = 1.0 / self.numParticles
		self.particles[i] = self.Particle(x, y, heading, weight)

	def LocalizeEstWithParticleFilter(self, encoder_measurements, sensor_readings):
		''' Localize the robot with particle filters. Call everything
			Args:
				delta_s (float): change in distance as calculated by odometry
				delta_heading (float): change in heading as calcualted by odometry
				sensor_readings([float, float, float]): sensor readings from range fingers
			Return:
				None'''

        # add student code here

		for i in range(self.numParticles):
			self.Propagate(encoder_measurements, i)
			self.particles[i].weight = self.CalculateWeight(sensor_readings,
															self.environment.walls,
															self.particles[i])
		self.Resample()

        # end student code here


		return self.GetEstimatedPos()

	def Propagate(self, encoder_measurements, i):
		'''Propagate all the particles from the last state with odometry readings
			Args:
				delta_s (float): distance traveled based on odometry
				delta_heading(float): change in heading based on odometry
			return:
				nothing'''
        # add student code here

		# Calculate difference in movement from last time step
		diffEncoder0 = +(encoder_measurements[0]-self.last_encoder_measurements[0])
		diffEncoder1 = -(encoder_measurements[1]-self.last_encoder_measurements[1])

		# At the first iteration, zero out
		if abs(diffEncoder0)> self.FAR_READING or abs(diffEncoder1)> self.FAR_READING:
			diffEncoder0 = 0
			diffEncoder1 = 0

		wheelDistanceL = -2 * math.pi * self.wheel_radius / self.encoder_resolution * diffEncoder0
		wheelDistanceR = 2 * math.pi * self.wheel_radius / self.encoder_resolution * diffEncoder1

		self.last_encoder_measurements[0] = encoder_measurements[0]
		self.last_encoder_measurements[1] = encoder_measurements[1]

		wheelDistanceL = wheelDistanceL + random.gauss(0, self.odom_xy_sigma)
		wheelDistanceR = wheelDistanceR + random.gauss(0, self.odom_xy_sigma)

		delta_s = 0.5 * (wheelDistanceR + wheelDistanceL)
		delta_theta = 0.5 / self.radius * (wheelDistanceR - wheelDistanceL)

		self.particles[i].x = self.particles[i].x + delta_s*math.cos(self.particles[i].heading+delta_theta/2)
		self.particles[i].y = self.particles[i].y + delta_s*math.sin(self.particles[i].heading+delta_theta/2)
		self.particles[i].heading = self.particles[i].heading + delta_theta

        # end student code here


	def CalculateWeight(self, sensor_readings, walls, particle):
		'''Calculate the weight of a particular particle
			Args:
				particle (E160_Particle): a given particle
				sensor_readings ( [float, ...] ): readings from the IR sesnors
				walls ([ [four doubles], ...] ): positions of the walls from environment,
							represented as 4 doubles
			return:
				new weight of the particle (float) '''

        # add student code here
		expectedMeasurements = [self.FindMinWallDistance(particle, walls, sensorT) for sensorT in self.sensor_orientation]
		newWeights = []
		for i in range(len(sensor_readings)):
			newWeight = (1 / (self.IR_sigma * (2 * math.pi)**0.5)) * math.exp(-0.5 * ((sensor_readings[i] - expectedMeasurements[i]) / self.IR_sigma)**2)
			newWeights.append(newWeight)

		return sum(newWeights) / len(newWeights)
        # end student code here

	def Resample(self):
		'''Resample the particles systematically
			Args:
				None
			Return:
				None'''
        # add student code here
		for i in range(self.numParticles):
			r = random.random()
			j = 0
			wsum = self.particles[0].weight
			while wsum < r and j < self.numParticles - 1:
				j += 1
				wsum += self.particles[j].weight
			self.particles[i] = self.particles[j]
        # end student code here




	def GetEstimatedPos(self):
		''' Calculate the mean of the particles and return it
			Args:
				None
			Return:
				None'''
		avgX = sum([particle.x for particle in self.particles])
		avgY = sum([particle.y for particle in self.particles])
		avgTheta = sum([particle.heading for particle in self.particles])
		newState = E160_state()
		newState.set_state(avgX, avgY, avgTheta)
		return newState


	def FindMinWallDistance(self, particle, walls, sensorT):
		''' Given a particle position, walls, and a sensor, find
			shortest distance to the wall
			Args:
				particle (E160_Particle): a particle
				walls ([E160_wall, ...]): represents endpoint of the wall
				sensorT: orientation of the sensor on the robot
			Return:
				distance to the closest wall (float)'''
		wallDistances = []
		for wall in walls:
			wallDistances.append(self.FindWallDistance(particle, wall, sensorT))

		return min(wallDistances)

	def euclidDist(self, p1, p2):
		return ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)**0.5

	def extractWall(self, p, wall):
		distProxy1 = self.euclidDist(p, (wall.points[0], wall.points[1])) + \
					 self.euclidDist(p, (wall.points[6], wall.points[7]))
		distProxy2 = self.euclidDist(p, (wall.points[2], wall.points[3])) + \
					 self.euclidDist(p, (wall.points[4], wall.points[5]))

		if distProxy1 < distProxy2:
			return wall.points[0], wall.points[1], wall.points[6], wall.points[7]
		else:
			return wall.points[2], wall.points[3], wall.points[4], wall.points[5]

	def crossProduct(self, p1, p2):
		return p1[0] * p2[1] - p1[1] * p2[0]

	def vectorDiff(self, p1, p2):
		return (p2[0] - p1[0], p2[1] - p1[1])

	def FindWallDistance(self, particle, wall, sensorT):
		''' Given a particle position, a wall, and a sensor, find distance to the wall
			Args:
				particle (E160_Particle): a particle
				wall ([float x4]): represents endpoint of the wall
				sensorT: orientation of the sensor on the robot
			Return:
				distance to the closest wall (float)'''
		# add student code here

        # Vectors for wall, points are q and q + s
		x1, y1, x2, y2 = self.extractWall((particle.x, particle.y), wall)
		q = (x1, y1)
		s = (x2 - x1, y2 - y1)

		# Vectors for sensor ray
		p = (particle.x, particle.y)
		r = (math.cos(sensorT + particle.heading),
			 math.sin(sensorT + particle.heading))

		rsproduct = self.crossProduct(r, s)

		if rsproduct == 0:
			# Lines are parallel
			return float('Inf')

		qpdiff = self.vectorDiff(p, q)
		t = self.crossProduct(qpdiff, s) / rsproduct
		u = self.crossProduct(qpdiff, r) / rsproduct

		if not (t >= 0 and u >= 0 and u <= 1):
			# Lines do not intersect
			return float('Inf')

		# intersection = (p[0] + t * r[0], p[1] + t * r[1])

		return ((t * r[0])**2 + (t * r[1])**2)**0.5



	def angleDiff(self, ang):
		''' Wrap angles between -pi and pi'''
		while ang < -math.pi:
			ang = ang + 2 * math.pi
		while ang > math.pi:
			ang = ang - 2 * math.pi
		return ang

	class Particle:
		def __init__(self, x, y, heading, weight):
			self.x = x
			self.y = y
			self.heading = heading
			self.weight = weight

		def __str__(self):
			return str(self.x) + " " + str(self.y) + " " + str(self.heading) + " " + str(self.weight)
