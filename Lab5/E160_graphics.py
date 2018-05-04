import math
from Tkinter import *
from E160_robot import *
from PIL import Image, ImageTk

class E160_graphics:
    
    def __init__(self, environment):
        self.environment = environment
        self.tk = Tk()
        #self.north_east_frame = Frame(self.tk)
        #self.north_east_frame.pack(anchor = NE)
        self.north_west_frame = Frame(self.tk)
        self.north_west_frame.pack(anchor = W)
        #self.north_frame = Frame(self.tk)
        #self.north_frame.pack(anchor = N)
        
        self.bottom_frame = Frame(self.tk)
        self.bottom_frame.pack(side = BOTTOM)
        
        self.scale = 200
        # self.canvas = Canvas(self.tk, width=self.environment.width*self.scale, height=self.scale* self.environment.height)
        self.canvas = Canvas(self.tk, width=600, height=600)

        self.tk.title("E160 - Autonomous Robot Navigation")
        self.canvas.bind("<Button-1>", self.callback)
        self.canvas.pack()
        self.gui_stopped = False
        self.last_rotate_control = 0
        self.last_forward_control = 0
        self.R = 0
        self.L = 0

        
        # add motor control slider
        self.forward_control = Scale(self.bottom_frame, from_=-100, to=100, length  = 400,label="Forward Control",tickinterval=50, orient=HORIZONTAL)
        self.forward_control.pack(side=LEFT)
        
        # add rotation control slider
        self.rotate_control = Scale(self.bottom_frame, from_=-100, to=100, length  = 400,label="Rotate Control",tickinterval=50, orient=HORIZONTAL)
        self.rotate_control.pack(side=RIGHT)
        
        # add track point button
        self.track_point_button = Button(self.bottom_frame, text="Track Point", anchor="s", wraplength=100, command=self.track_point).pack()
        
        # add stop button
        self.track_point_button = Button(self.bottom_frame, text="Stop", anchor="s", wraplength=100, command=self.stop).pack()
  
        # add stop button
        self.track_point_button = Button(self.bottom_frame, text="Quit", anchor="s", wraplength=100, command=self.quit).pack()
  
        # add range sensor measurements
        self.range_sensor_var_1 = StringVar()
        self.range_sensor_var_2 = StringVar()
        self.range_sensor_var_3 = StringVar()
        self.range_sensor_label_1 = Label(self.north_west_frame, textvariable = self.range_sensor_var_1).pack()
        self.range_sensor_label_2 = Label(self.north_west_frame, textvariable = self.range_sensor_var_2).pack()
        self.range_sensor_label_3 = Label(self.north_west_frame, textvariable = self.range_sensor_var_3).pack()

        # add encoder sensor measurements
        self.encoder_sensor_var_0 = StringVar()
        self.encoder_sensor_var_1 = StringVar()
        
        self.encoder_sensor_label_0 = Label(self.north_west_frame, textvariable = self.encoder_sensor_var_0).pack()
        self.encoder_sensor_label_1 = Label(self.north_west_frame, textvariable = self.encoder_sensor_var_1).pack()

        # add range sensor measurements
        self.x = StringVar()
        self.y = StringVar()
        self.theta = StringVar()
        self.x_label = Label(self.north_west_frame, textvariable = self.x).pack()
        self.y_label = Label(self.north_west_frame, textvariable = self.y).pack()
        self.theta_label = Label(self.north_west_frame, textvariable = self.theta).pack()
       
        # add text entry for desired X
        #self.x_des_label = Label(self.north_frame, text="X desired")
        #self.x_des_label.pack()
        self.x_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.x_des_entry.insert(10,"0.0")
        self.x_des_entry.pack()
        
        # add text entry for desired Y
        #self.y_des_label = Label(self.north_west_frame, text="Y desired")
        #self.y_des_label.pack()
        self.y_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.y_des_entry.insert(10,"0.0")
        self.y_des_entry.pack()
        
        # add text entry for desired Theta
        #self.theta_des_label = Label(self.north_west_frame, text="Theta desired")
        #self.theta_des_label.pack()
        self.theta_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.theta_des_entry.insert(10,"0.0")
        self.theta_des_entry.pack()
        
        # initilize particle representation
        self.particles_dot = [self.canvas.create_oval(0,0,0,0, fill ='black') for x in range(self.environment.robots[0].PF.numParticles)]
        self.do_draw_particles = False

        # draw static environment
        for w in self.environment.walls:
            self.draw_wall(w)
            
        # draw first robot
        for r in self.environment.robots:
            self.initial_draw_robot(r)    

        self.drawn_rrt = False
        self.RRT = []
        self.path = []

    def draw_rrt_help(self, node):
        # print node
        if node == None:
            pass
        else:
            for child in node.children:
                node_point = self.scale_points([node.x, node.y], self.scale)
                child_point = self.scale_points([child.x, child.y], self.scale)
                self.RRT.append(self.canvas.create_line(node_point, child_point, fill="gray"))
                self.draw_rrt_help(child)

    def draw_trajectory(self):
        for r in self.environment.robots:
            node_list = r.MP.node_list
            for i in range(len(r.MP.traj_node_list) - 1):
                next_node = node_list[r.MP.traj_node_list[i + 1]]
                current_node = node_list[r.MP.traj_node_list[i]]

                current_point = self.scale_points([current_node.x, current_node.y], self.scale)
                next_point = self.scale_points([next_node.x, next_node.y], self.scale)
                self.path.append(self.canvas.create_line(current_point, next_point, fill = "red"))

    def draw_rrt(self):
        if self.drawn_rrt == False:
            for branch in self.RRT:
                self.canvas.delete(branch)
            for path in self.path:
                self.canvas.delete(path)
            for r in self.environment.robots:
                robot_mp = r.MP
                head_node = r.MP.node_list[0]
                self.draw_rrt_help(head_node)
                self.draw_trajectory()
            self.drawn_rrt = True
        else:
            pass

            # self.canvas.create_line(head_node.x, head_node.y, )
    # def draw_RRT(self, node, children_index):
    #     if children_index == len()

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
        robot.tkimage = ImageTk.PhotoImage(robot.robot_gif.rotate(180/3.14*robot.state_draw.theta))
        robot.image = self.canvas.create_image(robot.state_draw.x, robot.state_draw.y, image=robot.tkimage)
        robot_points = self.scale_points([robot.state_draw.x, robot.state_draw.y], self.scale)
        self.canvas.coords(robot.image, *robot_points)
            
    def get_inputs(self):
        pass

    def draw_particles(self, robot):
        if self.do_draw_particles:
            for i in range(robot.PF.numParticles):
                pf_point = [robot.PF.particles[i].x, robot.PF.particles[i].y]
                point = self.scale_points(pf_point, self.scale)
                self.canvas.delete(self.particles_dot[i]) 
                self.particles_dot[i] = self.canvas.create_oval(point[0] - 2, point[1] - 2, point[0] + 2, point[1] + 2, fill =  'red')
            
    def track_point(self):
        self.environment.control_mode = "AUTONOMOUS CONTROL MODE"
                
        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.last_forward_control = 0
        self.last_rotate_control = 0
        self.R = 0
        self.L = 0
        
        # draw robots
        for r in self.environment.robots:
            x_des = float(self.x_des_entry.get())
            y_des = float(self.y_des_entry.get())
            theta_des = float(self.theta_des_entry.get())
            r.state_des.set_state(x_des,y_des,theta_des)
            r.point_tracked = False 
        
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
        robot.replan_path = True
        self.drawn_rrt = False
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
        
        
    def update_labels(self):
        
        self.range_sensor_var_1.set("Range 1 (m):  " + str(self.environment.robots[0].range_measurements[0]))
        self.range_sensor_var_2.set("Range 2 (m):  " + str(self.environment.robots[0].range_measurements[1]))
        self.range_sensor_var_3.set("Range 3 (m):  " + str(self.environment.robots[0].range_measurements[2]))
                
        self.encoder_sensor_var_0.set("Encoder 0 (m):  " + str(self.environment.robots[0].encoder_measurements[0]))
        self.encoder_sensor_var_1.set("Encoder 1 (m):  " + str(self.environment.robots[0].encoder_measurements[1]))

        self.x.set("X_est (m):  " + str(self.environment.robots[0].state_est.x))
        self.y.set("Y_est (m):  " + str(self.environment.robots[0].state_est.y))
        self.theta.set("Theta_est (rad):  " + str(self.environment.robots[0].state_est.theta))
        
        
        
    # called at every iteration of main loop
    def update(self):
        
        # update gui labels
        self.update_labels()
        
        # draw robots
        for r in self.environment.robots:
            self.draw_robot(r)     
        
        # draw particles
        self.draw_particles(self.environment.robots[0])
        
        # draw sensors
        
        # draw path
        self.draw_rrt()
 
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
        
        
        
    
    
    
   




