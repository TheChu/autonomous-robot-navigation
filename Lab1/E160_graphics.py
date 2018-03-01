import math
from Tkinter import *
from E160_robot import *
from PIL import Image, ImageTk

class E160_graphics:

    def __init__(self, environment):
        self.environment = environment
        self.tk = Tk()
        self.scale = 500
        self.canvas = Canvas(self.tk, width=self.environment.width*self.scale, height=self.scale* self.environment.height)
        self.tk.title("E160 - Autonomous Robot Navigation")
        self.canvas.bind("<Button-1>", self.callback)
        self.canvas.pack()
        self.gui_stopped = False
        self.last_rotate_control = 0
        self.last_forward_control = 0
        self.R = 0
        self.L = 0

        # add motor control slider
        self.forward_control = Scale(self.tk, from_=-100, to=100, length  = 400,label="Forward Control",tickinterval=50, orient=HORIZONTAL)
        self.forward_control.pack(side=LEFT)

        # add rotation control slider
        self.rotate_control = Scale(self.tk, from_=-100, to=100, length  = 400,label="Rotate Control",tickinterval=50, orient=HORIZONTAL)
        self.rotate_control.pack(side=RIGHT)

        # add track point button
        self.track_point_button = Button(self.tk, text="Track Point", anchor="s", wraplength=100, command=self.track_point).pack()

        # add stop button
        self.track_point_button = Button(self.tk, text="Stop", anchor="s", wraplength=100, command=self.stop).pack()

        # add stop button
        self.track_point_button = Button(self.tk, text="Quit", anchor="s", wraplength=100, command=self.quit).pack()

        # draw static environment
        for w in self.environment.walls:
            self.draw_wall(w)

        # draw first robot
        for r in self.environment.robots:
            self.initial_draw_robot(r)



    def draw_wall(self, wall):

        wall_points = self.scale_points(wall.points, self.scale)
        wall.poly = self.canvas.create_polygon(wall_points, fill='black')

    def scale_points(self, points, scale):
        scaled_points = []
        for i in range(len(points)-1):

            if i % 2 == 0:
                # for x values, just multiply times scale factor to go from meters to pixels
                scaled_points.append(self.environment.width/2*scale + points[i]*scale)

                # only flip y for x,y points, not for circle radii
                scaled_points.append(self.environment.height/2*scale - points[i+1]*scale)

        return scaled_points


    def reverse_scale_points(self, points, scale):
        reverse_scaled_points = []
        for i in range(len(points)-1):

            if i % 2 == 0:
                # for x values, just multiply times scale factor to go from meters to pixels
                reverse_scaled_points.append(-self.environment.width/2 + points[i]/scale)

                # only flip y for x,y points, not for circle radii
                reverse_scaled_points.append(self.environment.height/2 - points[i+1]/scale)

        return reverse_scaled_points


    def initial_draw_robot(self, robot):

        # open image
        robot.robot_gif = Image.open("E160_robot_image.gif").convert('RGBA')


    def draw_robot(self, robot):

        # gif update
        robot.tkimage = ImageTk.PhotoImage(robot.robot_gif.rotate(180/3.14*robot.state.theta))
        robot.image = self.canvas.create_image(robot.state.x, robot.state.y, image=robot.tkimage)
        robot_points = self.scale_points([robot.state.x, robot.state.y], self.scale)
        self.canvas.coords(robot.image, *robot_points)

    def get_inputs(self):
        pass



    def track_point(self):
        self.environment.control_mode = "AUTONOMOUS CONTROL MODE"

        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.last_forward_control = 0
        self.last_rotate_control = 0
        self.R = 0
        self.L = 0

    def stop(self):
        self.environment.control_mode = "MANUAL CONTROL MODE"

        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.last_forward_control = 0
        self.last_rotate_control = 0
        self.R = 0
        self.L = 0

    def quit(self):
        self.environment.control_mode = "MANUAL CONTROL MODE"
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.gui_stopped = True


    def callback(self, event):
        desired_points = self.reverse_scale_points([float(event.x), float(event.y)], self.scale)
        robot = self.environment.robots[0]
        robot.state_des.set_state(desired_points[0],desired_points[1],0)
        print "New desired robot state", robot.state_des.x, robot.state_des.y


    def send_robot_commands(self):

        # check to see if forward slider has changed
        if abs(self.forward_control.get()-self.last_forward_control) > 0:
            self.rotate_control.set(0)
            self.last_forward_control = self.forward_control.get()
            self.last_rotate_control = 0
            self.environment.control_mode = "MANUAL CONTROL MODE"

            # extract what the R and L motor signals should be
            self.R = self.forward_control.get()
            self.L = self.forward_control.get()

        # check to see if rotate slider has changed
        elif abs(self.rotate_control.get()-self.last_rotate_control) > 0:
            self.forward_control.set(0)
            self.last_rotate_control = self.rotate_control.get()
            self.last_forward_control = 0
            self.environment.control_mode = "MANUAL CONTROL MODE"

            # extract what the R and L motor signals should be
            self.R = -self.rotate_control.get()
            self.L = self.rotate_control.get()

        # if manual mode, set motors
        if self.environment.control_mode == "MANUAL CONTROL MODE":

            # tell robot what the values should be
            robot = self.environment.robots[0]
            robot.set_manual_control_motors(self.R, self.L)





    # called at every iteration of main loop
    def update(self):

        # draw robots
        for r in self.environment.robots:
            self.draw_robot(r)

        # draw particles


        # draw sensors


        # update the graphics
        self.tk.update()

        # check for gui buttons
        self.get_inputs()

        # send commands to robots
        self.send_robot_commands()

        # check for quit
        if self.gui_stopped:
            self.environment.quit()
            return False
        else:
            return True
