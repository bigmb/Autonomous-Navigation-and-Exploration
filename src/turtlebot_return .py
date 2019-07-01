#!/usr/bin/env python

# ROS imports
import roslib; roslib.load_manifest('autonomous_explore_map_plan')
import rospy
import tf
import math
import time
#import the library to compute transformations
from tf.transformations import euler_from_quaternion
from probabilistic_lib.functions import angle_wrap #Normalize angles between -pi and pi


#ROS messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelStates
from autonomous_explore_map_plan.srv import GotoWaypoint, GotoWaypointResponse, FindPathToGoal, FindPathToGoalResponse, FindPathToGoalRequest

#Numpy
import numpy as np

class Returner(object):

    def __init__(self):
        #rospy.init_node('turtlebot_drive', log_level=rospy.INFO)
        #rospy.loginfo("%s: starting turtlebot controller", rospy.get_name())
        self.current_position_ = np.zeros(2)
        self.current_orientation_ = 0.0
        
        self.desired_position_ = np.zeros(2)
        self.desired_orientation_ = 0.0
        self.vmsg=Twist()
        self.goal_th_xy = 0.1
        self.goal_th_ang = 0.01



        self.odometry_sub_ = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size = 1)
        #self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.modelcallback)
        self.map_sub_ = rospy.Subscriber("/projected_map", OccupancyGrid, self.OccupancyGridCallback, queue_size = 1)
        self.control_input_pub_ = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size = 10)
        
        self.serv_ = rospy.Service('/turtlebot_return/goto', 
                                  GotoWaypoint, 
                                  self.calculateControlInput2)
        
        
        
        
        # rotate once at the beginning before exploring
        #rospy.sleep(1)
        #self.rotateOnce()
        
        #print ('before service wait')
        rospy.wait_for_service('/turtlebot_return/find_path_to_goal')
        try:
            self.find_path_to_goal_serv_2 = rospy.ServiceProxy('/turtlebot_return/find_path_to_goal', FindPathToGoal)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        return
        #print ('after service wait')

    '''
    def service(self):
        print ('in service function')
        self.serv_ = rospy.Service('/turtlebot_drive/goto', 
                                  GotoWaypoint, 
                                  self.calculateControlInput)
        rospy.wait_for_service('/turtlebot_drive/find_path_to_goal')
        try:
            self.find_path_to_goal_serv_ = rospy.ServiceProxy('/turtlebot_drive/find_path_to_goal', FindPathToGoal)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    '''
    def calculateControlInput2(self, req):
    
        GotoResp = GotoWaypointResponse()
        #print ('in controlinput2')
        planner_request = FindPathToGoalRequest()
        planner_request.goal_state_x = req.goal_state_x
        planner_request.goal_state_y = req.goal_state_y

        print ('in turtlebot_return function')
        print ('currentx' + str(self.current_position_[0]))
        print ('currenty' + str(self.current_position_[1]))
        print ('currentx' + str(req.goal_state_x))
        print ('currenty' + str(req.goal_state_y))
        planner_response = self.find_path_to_goal_serv_2(planner_request)

        checker = 0

        #GotoResp.FindPathFlag = 0
        for pose in planner_response.poses:
            self.desired_position_[0] = pose.x
            self.desired_position_[1] = pose.y
            
            check_skip = 0
            r = rospy.Rate(30)
            while True:
                self.compute_velocity() # and publish
                #rospy.sleep(0.5)

                check_skip = (check_skip+1) % 4
                if check_skip == 0:
                    obstacle = self.check_unknown_obstacle()
                    if obstacle is True:
                        print ('obstacle in path')
                        GotoResp.FindPathFlag = 0
                        return GotoResp

                xy_reach = self.has_arrived_xy()
                if xy_reach:
                    print ('goal reached')
                    #GotoResp.FindPathFlag = 1
                    checker += 1
                    break

                r.sleep()
            
            #if checker == 2:
                #break
        #return GotoWaypointResponse()
        GotoResp.FindPathFlag = 1

        return GotoResp
            
    def check_unknown_obstacle(self):
        # 1. defined unit vector from current pos to goal pos (gazebo unit)
        # 2. define segments in gazebo unit to assess obstacle
        # 3. for i = all seg, convert each to cell unit and check value
        dist_to_goal_xy = self.dist_to_goal_xy()
        unit_distance = 0.1

        x_unit_vect = (self.desired_position_[0] - self.current_position_[0])*1.2 #/ dist_to_goal_xy
        y_unit_vect = (self.desired_position_[1] - self.current_position_[1])*1.2 #/ dist_to_goal_xy
        num_seg = int(round(dist_to_goal_xy / unit_distance))
        obstacle = False

        data1 = np.reshape(self.dat, (self.wid ,self.heigh), order="F")
        data2 = np.asarray(data1)
        x_org = self.xorg
        y_org = self.yorg
        x = self.current_position_[0]
        y = self.current_position_[1]

        
        for i in range(1, num_seg+1):
            x_gazebo =  x + x_unit_vect/num_seg * i
            y_gazebo =  y + y_unit_vect/num_seg * i

            x_map, y_map = self.gazebo2map(x_gazebo, y_gazebo, x_org, y_org)

        #print ('x_map is '+str(x_map))
            #print ('y_map is '+str(y_map))

            #data1 = np.reshape(self.dat, (self.wid ,self.heigh), order="F")
            #data2 = np.asarray(data1)
        
        #print ('in the map:'+str(data2[x_map, y_map]))

            # window around (x_map, y_map) all will be checked
            win = np.zeros((5, 5)) 
            win = data2[x_map-2:x_map+3, y_map-2:y_map+3]
            if np.sum([win]) >= 100:
            #if data2[x_map, y_map] == 100: # obstacle
                obstacle = True
                return obstacle
                 #break
        return obstacle

    def gazebo2map(self, x_gazebo, y_gazebo, x_org, y_org):
        # convert units from gazebo to map
        x_map = int(math.floor((x_gazebo - x_org) / self.res))
        y_map = int(math.floor((y_gazebo - y_org) / self.res))
        #print ('x_map is '+str(x_map))
            #print ('y_map is '+str(y_map))
            #print ('x_org is '+str(self.xorg))
            #print ('y_org is '+str(self.yorg))
        #print ('x_gazebo is '+str(x_gazebo))
            #print ('y_gazebo is '+str(y_gazebo))
        #print ('height is '+str(self.heigh))
            #print ('width is '+str(self.wid))

        if x_map < 0:
            x_map = 0
        elif x_map > self.wid - 2:
            x_map = self.wid -1

        if y_map < 0:
            y_map = 0
        elif y_map > self.heigh    -1:
            y_map = self.heigh -1


        #print ('x_map is '+str(x_map))
            #print ('y_map is '+str(y_map))
            #print ('x_org is '+str(self.xorg))
            #print ('y_org is '+str(self.yorg))
        #print ('x_gazebo is '+str(x_gazebo))
            #print ('y_gazebo is '+str(y_gazebo))
        #print ('height is '+str(self.heigh))
            #print ('width is '+str(self.wid))
        
        return x_map, y_map        

    def dist_to_goal_xy(self):
        
            #dist_to_goal_xy computes the distance in x and y direction to the 
            #active goal
        
            return math.sqrt(pow(self.current_position_[0]-self.desired_position_[0],2)+pow(self.current_position_[1]-self.desired_position_[1],2))
            

    def dist_to_goal_ang(self):
        
        #dist_to_goal_ang computes the orientation distance to the active
        #goal
        
            return wrapAngle(self.desired_orientation_-self.current_orientation_)    


    def has_arrived_xy(self):
        
        #has_arrived_xy returns true if the xy distance to the ative goal is
        #smaller than the position threshold
        
            return self.dist_to_goal_xy()<self.goal_th_xy
    
    def compute_velocity2(self):
        # Build new message in temporal variable
        self.vmsg = Twist()

        if not self.follow_path:
        # Smoothly but quickly break (To prevent drifting)
        # Round to 2 decs to avoid small residual speeds
            temp_msg.linear.x = np.round(self.vmsg.linear.x * 0.4, decimals=2)  
            temp_msg.angular.z = np.round(self.vmsg.angular.z * 0.4, decimals=2)

        elif self.dist_to_goal_xy() > self.goal_th_xy:
        # Compute delta_angle
            angle_to_goal = math.atan2(self.goals_y[self.active_goal] - self.position_y, self.goals_x[self.active_goal]-self.position_x)
            delta_angle_to_goal = angle_wrap(self.position_theta - angle_to_goal)

        # Angular speed
            sign_angle = delta_angle_to_goal/np.abs(delta_angle_to_goal)

            temp_msg.angular.z = -sign_angle*min(self.max_ang_speed,                  # Max value
                                             6*np.abs(delta_angle_to_goal),       # Proportional component
                                             abs(self.vmsg.angular.z)*1.1 + 0.05) # Smoothing factor

        # Linear speed
            if np.abs(delta_angle_to_goal) < 0.1:
                temp_msg.linear.x = min(self.max_lin_speed,              # Max value
                                    2*self.dist_to_goal_xy() + 0.1,  # Proportional component
                                    self.vmsg.linear.x*1.1 + 0.05)   # Smoothing factor
            else:
                temp_msg.linear.x = self.vmsg.linear.x*0.75 #Keep some speed if not facing obstacle


        self.vmsg = temp_msg

    def compute_velocity(self):
        
        #compute_velocity computes the velocity which will be published.
        
        #TODO implement!
         
        # Build new message in temporal variable
        #self.vmsg = Twist()
        self.max_lin_speed = 0.8
        self.max_ang_speed = 0.8 



        if self.dist_to_goal_xy() > self.goal_th_xy:
                # Compute delta_angle
                angle_to_goal = math.atan2(self.desired_position_[1] - self.current_position_[1], self.desired_position_[0]-self.current_position_[0])
        #print ('angle to goal angle before wrap:'+ str(angle_to_goal))
                delta_angle_to_goal = wrapAngle(self.current_orientation_ - angle_to_goal) #wrapAngle

                # Angular speed
                sign_angle = delta_angle_to_goal/np.abs(delta_angle_to_goal)
        
                self.vmsg.angular.z = -sign_angle*min(self.max_ang_speed,                  # Max value
                                                 6*np.abs(delta_angle_to_goal),       # Proportional component
                                                 abs(self.vmsg.angular.z)*1.1 + 0.05) # Smoothing factor
                print ('delta angle:'+ str(delta_angle_to_goal))
        #print ('current orientation angle:'+ str(self.current_orientation_))
        #print ('angle to goal angle:'+ str(angle_to_goal))
                # Linear speed
                if np.abs(delta_angle_to_goal) < 0.1:
                    self.vmsg.linear.x = min(self.max_lin_speed,              # Max value
                                    2*self.dist_to_goal_xy() + 0.1,  # Proportional component
                                    self.vmsg.linear.x*1.1+ 0.05)  # Smoothing factor
                else:
                    self.vmsg.linear.x = self.vmsg.linear.x*0.75 #Keep some speed if not facing obstacle


            #self.vmsg = temp_msg
        
        self.control_input_pub_.publish(self.vmsg)

        '''
        if self.dist_to_goal_xy() >= self.goal_th_xy:
            # Compute delta_angle
            angle_to_goal = math.atan2(self.desired_position_[1] - self.current_position_[1], self.desired_position_[0]-self.current_position_[0])
            delta_angle_to_goal = wrapAngle(self.current_orientation_ - angle_to_goal)
        print ('delta angle:'+ str(delta_angle_to_goal))
        print ('current orientation angle:'+ str(self.current_orientation_))
        print ('current orientation angle:'+ str(angle_to_goal))
        #np.abs(delta_angle_to_goal) >= 0.05:
            # Assign angular velocity as 2*delta_angle
            self.vmsg.angular.z = -1.2*delta_angle_to_goal #-1.5
            
            # If delta_angle small (facing obstacle, we can move)
            if np.abs(delta_angle_to_goal) < 0.02:
                
                self.vmsg.linear.x = min(0.5, 2*self.dist_to_goal_xy())
        '''
            #else:    
                #self.vmsg.angular.z = -1.2*self.dist_to_goal_ang()#-1.2
        
             
        
    

    def odomCallback(self, odometry_msg):
        self.current_position_[0] = odometry_msg.pose.pose.position.x
        self.current_position_[1] = odometry_msg.pose.pose.position.y
        (r, p, y) = tf.transformations.euler_from_quaternion([odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w])
        self.current_orientation_ = wrapAngle(y)
        return

    def OccupancyGridCallback(self, msg):            
        self.dat = msg.data
        self.wid = msg.info.width
        self.heigh = msg.info.height
        self.res = msg.info.resolution
        self.xorg = msg.info.origin.position.x
        self.yorg = msg.info.origin.position.y


    def modelcallback(self, msg):
        #a = model_msg.name[6]
        #mobile_base_idx = 6
        #print (str(model_msg.pose[mobile_base_idx].orientation.x))
        #print (str(len(model_msg.name)))
        mobile_base_idx = -1
        for i in range(len(msg.name)):
            #print ('in loop')
                if msg.name[i] == 'mobile_base':
                    mobile_base_idx = i
                #print (str(mobile_base_idx))
                    break
        #print (str(mobile_base_idx))

        self.current_position_[0] = msg.pose[mobile_base_idx].position.x
        self.current_position_[1] = msg.pose[mobile_base_idx].position.y
        (ang_x, ang_y, ang_z) = euler_from_quaternion([msg.pose[mobile_base_idx].orientation.x,
                                                msg.pose[mobile_base_idx].orientation.y,
                                                msg.pose[mobile_base_idx].orientation.z,
                                                msg.pose[mobile_base_idx].orientation.w]) 
        #Update theta with euler_z
        self.current_orientation_ = wrapAngle(ang_z)
        #return


    def rotateOnce(self):
        print ('current orientation' + str(self.current_orientation_))
        control_input = Twist()
        control_input.angular.z = 0.6
        #rate = rospy.Rate(10) # 10hz
        while np.abs(self.current_orientation_) < 0.5:
            #control_input.angular.z  = control_input.angular.z * 1.1 + 0.05
            self.control_input_pub_.publish(control_input)

        #rospy.sleep(1.0)
        rotate = 0
        while True:
            #control_input.angular.z  = control_input.angular.z * 1.1 + 0.05
            self.control_input_pub_.publish(control_input)
            #print ('current orientation' + str(self.current_orientation_))
            #rospy.sleep(0.1)
            if np.abs(self.current_orientation_) < 0.1:
                rotate += 1
                print ('rotate' + str(rotate))
            if rotate == 1:
                break
         

def angle_wrap(angle): #angle_wrap
    """wrapAngle
    
    Calculates angles values between 0 and 2pi"""
    return (angle + ( 2.0 * math.pi * math.floor( ( math.pi - angle ) / ( 2.0 * math.pi ) ) ) )


def wrapAngle(ang):
    """
    Return the angle normalized between [-pi, pi].

    Works with numbers and numpy arrays.

    :param ang: the input angle/s.
    :type ang: float, numpy.ndarray
    :returns: angle normalized between [-pi, pi].
    :rtype: float, numpy.ndarray
    """
    ang = ang % (2 * np.pi)
    if (isinstance(ang, int) or isinstance(ang, float)) and (ang > np.pi):
        ang -= 2 * np.pi
    elif isinstance(ang, np.ndarray):
        ang[ang > np.pi] -= 2 * np.pi
    return ang

if __name__ == '__main__':
    rospy.init_node('turtlebot_return', log_level=rospy.INFO)
    rospy.loginfo("%s: starting turtlebot returner", rospy.get_name())

    print ('before controller')
    returner = Returner()
    print ('after returner')
    #print (str(returner.current_orientation_))
    #while not rospy.is_shutdown():
    #controller.rotateOnce()
        
    #controller.service()
    #print ('exit out drive')
    rospy.spin()

