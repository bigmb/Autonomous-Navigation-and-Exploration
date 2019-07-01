#!/usr/bin/env python

# ROS imports
import roslib; roslib.load_manifest('autonomous_explore_map_plan')
import rospy
import tf
import math

#ROS messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from autonomous_explore_map_plan.srv import GotoWaypoint, GotoWaypointResponse, GotoWaypointRequest, FindPathToGoal, FindPathToGoalResponse, FindPathToGoalRequest

#Numpy
import numpy as np

from MapProcessing import ProcessMap
from turtlebot_drive import Controller
from turtlebot_return import Returner


global initial_position_
initial_position_ = np.zeros(2)


def odomCallback(odometry_msg):
        global initial_position_
        initial_position_[0] = odometry_msg.pose.pose.position.x
        initial_position_[1] = odometry_msg.pose.pose.position.y
        return


if __name__ == '__main__':
    rospy.init_node('explore_node', log_level=rospy.INFO)
    rospy.loginfo("%s: starting turtlebot exploration", rospy.get_name())

    #controller = Controller()
    #print (str(controller.current_orientation_))
    #while not rospy.is_shutdown():
    #controller.rotateOnce()

    # get map from controller through service call
    #self.map_sub_ = rospy.Subscriber("/projected_map", OccupancyGrid, self.occupCallback, queue_size = 1)

    # pass the map and its resolution to ProcessProjectedMap()
    global initial_position_
    odometry_sub_ = rospy.Subscriber("/odom", Odometry, odomCallback, queue_size = 1)
    rospy.sleep(10)
    
    initial_position_local = np.zeros(2)
    initial_position_local[0] = initial_position_[0]
    initial_position_local[1] = initial_position_[1]


    print ('before return service')
    rospy.wait_for_service('/turtlebot_return/goto')
    try:
        goto_serv_ = rospy.ServiceProxy('/turtlebot_return/goto', GotoWaypoint)
    except rospy.ServiceException, e:
        print "Goto Service call failed: %s"%e


    print ('after return service')
    rospy.sleep(10)
    start_time = rospy.Time.now()

    three_min_timeout = rospy.Duration(240)
    #while rospy.Time.now() - start_time < three_min_timeout:
        #print (str(rospy.Time.now() - start_time))

    while rospy.Time.now() - start_time < three_min_timeout:
        
        pm = ProcessMap()
        pm.ProcessProjectedMap()
        print ('enter loop again')
    #ProcessMap().ProcessProjectedMap()

    print ('exit out explore, begin to return')
   
    
    
    while True:
        print('in explorer node')
        print('returning to ' + 'x:'+str(initial_position_local[0]))
        print('returning to ' + 'y:'+str(initial_position_local[1]))
        return_request = GotoWaypointRequest()
        return_request.goal_state_x = initial_position_local[0]
        return_request.goal_state_y = initial_position_local[1]
                
        return_response = goto_serv_(return_request)

        if return_response.FindPathFlag == 1:
            break

        print ('returning again')

    print ('finish return')
    rospy.spin()
