import math

class E160_state:

    def __init__(self, x = 0.0, y = 0.0, theta = 0.0):
        self.set_state(x,y,theta)
        
    def set_state(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __add__(self, other):
        return E160_state(self.x + other.x, self.y + other.y, self.angle_wrap(self.theta + other.theta))

    def __sub__(self, other):
        return E160_state(self.x - other.x, self.y - other.y, self.angle_wrap(self.theta - other.theta))

    def __repr__(self):
    	return "[" + str(self.x) + " " + str(self.y) + " " + str(self.theta) + "]"

    def xydist(self, other):
        return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

    def angle_wrap(self, a):
        while a > math.pi:
            a = a - 2*math.pi
        while a < -math.pi:
            a = a + 2*math.pi
        return a
