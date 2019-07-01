#!/usr/bin/env python

# ROS general imports
import roslib
roslib.load_manifest('autonomous_explore_map_plan')
import rospy

# Other imoprts
import math #math library
import numpy as np #numpy library
from probabilistic_lib.functions import angle_wrap #Normalize angles between -pi and pi

#import the library to read csv
import csv

#import the library to compute transformations
from tf.transformations import euler_from_quaternion

#ROS messages
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist


class driver(object):
    
    def __init__(self):
        '''
        constructor
        '''
        #Initialize ros node
        rospy.init_node('turtlebot_driver')        
        
        #Initialize goals
        self.x = np.array([])
        self.y = np.array([])
        self.theta = np.array([])
        
        #Threshold for distance to goal
        self.goal_th_xy = rospy.get_param('goal_thershold_xy',0.1) #Position threshold
        self.goal_th_ang = rospy.get_param('goal_threshold_ang',0.01) #Orientation threshold
        
        #Point to the first goal
        self.active_goal = 0
        
        #Define subscriber
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        
        #Define publisher either /mobile_base/commands/velocity or /cmd_vel_mux/input/teleop        
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        
        #Define the velocity message
        self.vmsg = Twist()     
        
        #Has the goal been loaded?
        self.params_loaded = False            

    def print_goals(self):
        '''
        print_goals prints the list of goals
        '''
        rospy.loginfo("List of goals:")
        rospy.loginfo("X:\t " + str(self.x))
        rospy.loginfo("Y:\t " + str(self.y))
        rospy.loginfo("Theta:\t " + str(self.theta))
    
    def print_goal(self):
        '''
        pritn_goal prints the next goal
        '''
        rospy.loginfo( "Goal: (" + str(self.x[self.active_goal]) + " , " + str(self.y[self.active_goal]) + " , " + str(self.theta[self.active_goal]) + ")")
        
    def print_pose(self):
        '''
        print_pose pints the robot's actuall position
        '''
        rospy.loginfo( "Pose: (" + str(self.position_x) + " , " + str(self.position_y) + " , " + str(self.position_theta) + " )")
        
    def print_goal_and_pose(self):
        '''
        pritn_goal_and_pose prints both goal and pose
        '''
        rospy.loginfo("\tPose\t\tGoal")
        rospy.loginfo("X:\t%f\t%f",self.position_x,self.x[self.active_goal])
        rospy.loginfo("Y:\t%f\t%f",self.position_y,self.y[self.active_goal])
        rospy.loginfo("A:\t%f\t%f",self.position_theta,self.theta[self.active_goal])
        
    def dist_to_goal_xy(self):
        '''
        dist_to_goal_xy computes the distance in x and y direction to the 
        active goal
        '''
        return math.sqrt(pow(self.position_x-self.x[self.active_goal],2)+pow(self.position_y-self.y[self.active_goal],2))
    
    def dist_to_goal_ang(self):
        '''
        dist_to_goal_ang computes the orientation distance to the active
        goal
        '''
        return angle_wrap(self.theta[self.active_goal]-self.position_theta)
        
    def has_arrived_xy(self):
        '''
        has_arrived_xy returns true if the xy distance to the ative goal is
        smaller than the position threshold
        '''
        return self.dist_to_goal_xy()<self.goal_th_xy
        
    def has_arrived_ang(self):
        '''
        has_arrived_ang returns true if the orientation distance to the 
        ative goal is smaller than the orientation threshold
        '''
        return self.dist_to_goal_ang()<self.goal_th_ang
        
    def has_arrived(self):
        '''
        has_arrived return true if the robot is closer than the apropiate 
        threshold to the goal in position and orientation
        '''
        return (self.has_arrived_xy() and self.has_arrived_ang())   
    
    def check_goal(self):
        '''
        check_goal checks if the robot has arrived to the active goal, 
        '''
        if self.has_arrived():
            self.next_goal()
        
    def publish(self):
        '''
        publish publish the velocity message in vmsg
        '''
        self.pub.publish(self.vmsg)
        
    def callback(self,msg):
        '''
        callback reads the actuall position of the robot, computes the 
        appropiate velocity, publishes it and check if the goal is reached
        '''
        self.read_position(msg)
        self.compute_velocity()
        self.publish()
        self.check_goal()
        
    def drive(self):
        '''
        drive is a neede function for the ros to run untill somebody stops
        the driver
        '''
        self.print_goal()
        while not rospy.is_shutdown():
            rospy.sleep(0.03)
        
    def load_goals(self):
        '''
        load_goals loads the goal (or goal list for the option al part) into
        the x y theta variables.
        '''
        if rospy.has_param("goals_file"):
            with open(rospy.get_param("goals_file"), 'rb') as csvfile:
        		reader = csv.reader(csvfile)
        		for row in reader:
        			self.x = np.append( self.x, float(row[0]))
	        		self.y = np.append( self.y, float(row[1]))
	        		self.theta = np.append( self.theta, float(row[2]))	
        else:
	        self.x = np.append( self.x, rospy.get_param('x',0))
	        self.y = np.append( self.y, rospy.get_param('y',0))
	        self.theta = np.append( self.theta,rospy.get_param('theta',0))

        self.params_loaded = True
        self.print_goals()
        
    def next_goal(self):
        '''
        next_goal increments the index of the goal in 1 and checks whether
        or not the robot has reached the final goal
        '''

        self.active_goal += 1

        if(self.active_goal >= len(self.x)):
        	print('Final goal reached!')
        	rospy.signal_shutdown('Final goal reached!')
        else:
        	self.print_goal()
        
    def read_position(self,msg):
        '''
        read_position copy the position received in the message (msg) to the
        internal class variables self.position_x, self.position_y and 
        self.position_theta
        
        Tip: in order to convert from quaternion to euler angles give a look
        at tf.transformations
        '''

        #Find index of mobile_base
        mobile_base_idx = -1
        for i in range(len(msg.name)):
        	if msg.name[i] == 'mobile_base':
        		mobile_base_idx = i
        		break

        #Update position_x and position_y
        self.position_x = msg.pose[mobile_base_idx].position.x
        self.position_y = msg.pose[mobile_base_idx].position.y

        #Get euler angles from quaternion
        (ang_x, ang_y, ang_z) = euler_from_quaternion([msg.pose[mobile_base_idx].orientation.x,
                                                    msg.pose[mobile_base_idx].orientation.y,
                                                    msg.pose[mobile_base_idx].orientation.z,
                                                    msg.pose[mobile_base_idx].orientation.w]) 
        #Update theta with euler_z
        self.position_theta = angle_wrap(ang_z)

        
    def compute_velocity(self):
        '''
        compute_velocity computes the velocity which will be published.
        
        TODO implement!
        '''
        self.vmsg=Twist()  

        if self.dist_to_goal_xy() > self.goal_th_xy:
        	# Compute delta_angle
        	angle_to_goal = math.atan2(self.y[self.active_goal] - self.position_y, self.x[self.active_goal]-self.position_x)
        	delta_angle_to_goal = angle_wrap(self.position_theta - angle_to_goal)

	        # Assign angular velocity as 2*delta_angle
	        self.vmsg.angular.z = -delta_angle_to_goal

	        # If delta_angle small (facing obstacle, we can move)
	        if np.abs(delta_angle_to_goal) < 0.05:
	            self.vmsg.linear.x = min(0.5, 2*self.dist_to_goal_xy())
        else:	
            self.vmsg.angular.z = -self.dist_to_goal_ang()

        	
