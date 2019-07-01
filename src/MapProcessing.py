#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import scipy.io as sio
import scipy.ndimage as ndimage 
import rospy
import time
import math
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from autonomous_explore_map_plan.srv import GotoWaypoint, GotoWaypointRequest, GotoWaypointResponse
np.set_printoptions(threshold='nan')


class ProcessMap(object):
        def __init__(self):
                self.odometry_sub_ = rospy.Subscriber("/odom",Odometry, self.OdometryCallback)
                self.current_position = np.zeros(2)

                # get the map >> call ProcessProjectedMap() 
                self.map_sub_ = rospy.Subscriber("/projected_map", OccupancyGrid, self.OccupancyGridCallback, queue_size = 1)
                
                # Use RRT* to go to best point 
                rospy.wait_for_service('/turtlebot_drive/goto')
                try:
                        self.goto_serv_ = rospy.ServiceProxy('/turtlebot_drive/goto', GotoWaypoint)
                except rospy.ServiceException, e:
                        print "Goto Service call failed: %s"%e
                return


        def OccupancyGridCallback(self, msg):                   
                self.dat = msg.data
                self.wid = msg.info.width
                self.heigh = msg.info.height
                self.res = msg.info.resolution
                self.xorg = msg.info.origin.position.x
                self.yorg = msg.info.origin.position.y 

        def OdometryCallback(self, msg):
                self.current_position[0] = msg.pose.pose.position.x
                self.current_position[1] = msg.pose.pose.position.y

        
        def FindBestPoint(self, BrushMap):
                rows = BrushMap.shape[0]
                cols = BrushMap.shape[1]
                print(rows)
                print(cols)
                
                NumUnknownElems = np.count_nonzero(BrushMap == -1)
  		NumWallElems = np.count_nonzero(BrushMap == 100)
  		#best = 109
  		#NumbestElems = np.count_nonzero(BrushMap == best)
  		

  		if (NumWallElems > 1):
  			BestGazeboArray = self.WallsFreeUnknown(BrushMap)

  		#return bestPoint, BestGazebo 
  		return BestGazeboArray

             
        def WallsFreeUnknown(self, BrushMap):
        	rows = BrushMap.shape[0]
  		cols = BrushMap.shape[1]
		# this gives you the highest value
                #best=np.amax(BrushMap)
                #print(best)
                best = 106
                
                if best > np.amax(BrushMap):
                    best = np.amax(BrushMap)
                    
                window = 10
                index=[]
		# finding indeces of maximum values
                for k in range(window, rows-window+1):
                        for j in range(window, cols-window+1):
                                if BrushMap[k][j]==best:
                                        inr=[k,j]
                                        index.append(inr)             
                
                index = np.asarray(index) 
                
                N = []
                for i in range(index.shape[0]):
                        n = 0
                        for k in range(-window, window):
                                for j in range(-window, window):
                                        if BrushMap[index[i][0]+k][index[i][1]+j] == -1:
                                                n=n+1
                        N.append(n) 
                
                N = np.asarray(N)
                #max_value = max(N)
                #max_index = N.index(max_value)  
                        
                num = 20
                if index.shape[0] <  num:
                	num = index.shape[0]
                	
                # getting indeces of best 5 points
                
                Nlargest = N.argsort()[-num:]
                Nlargest = np.asarray(Nlargest)        
                             
                SmallestDist = []
                for m in range(Nlargest.shape[0]):
                	#print('Brushmap indeces: ')
                	#print( np.asarray([index[Nlargest[m]][0], index[Nlargest[m]][1]]) )
                	dist = self.dist_to_bestpoint_xy(np.asarray([index[Nlargest[m]][0], index[Nlargest[m]][1]]) )
                	SmallestDist.append(dist)
                	
                # sort the array by the smallest dist to robot
                SmallestDist = np.asarray(SmallestDist)
                DistSorted = SmallestDist.argsort()
                
                print(SmallestDist)
                print(DistSorted)
                
                BestGazeboArray = np.zeros((num,2))*1.0
                
                for z in range(DistSorted.shape[0]):
                	max_index = Nlargest[DistSorted[z]]                                            
                	#max_value = max(N)
                	#max_index = N.index(max_value)          
                	bestPoint=[[index[max_index][0],index[max_index][1]]]
                	BestGazebo=[[float(index[max_index][0]*self.res+self.xorg),float(index[max_index][1]*self.res+self.yorg)]]
                	BestGazeboArray[z][0] = BestGazebo[0][0]
                	BestGazeboArray[z][1] = BestGazebo[0][1]
                	
                
                #print('Best point is ' )
                #print(bestPoint)
                print('best points in Gazebo is')
                print(BestGazeboArray)
                #bestPoint = np.array(bestPoint)
                BestGazebo = np.array(BestGazebo)

                return BestGazeboArray
                #return bestPoint, BestGazebo
                
        def dist_to_bestpoint_xy(self, desired_position ):
            # computes the distance in x and y direction to the given goal
        
            return math.sqrt(pow(self.current_position[0]-desired_position[0],2)+pow(self.current_position[1]-desired_position[1],2))
            
             



        def brushfire(self, map1):
                #print(map1)
                
                rows = map1.shape[0]
                cols = map1.shape[1]
                num_zeros = (map1 == 0).sum()
                print('size of map start brushfire: ')
                print(map1.shape)

                k=100; #Obstacle value
                while num_zeros >0 :
                        for i in range(rows):
                                for j in range(cols):
                                        if map1[i][j]==k:
                                                if (i-1)>=0 and (i+1)<rows and (j-1)>=0 and (j+1)<cols:
                                                        if map1[i-1][j-1]==0:
                                                            map1[i-1][j-1]=k+1
                                                            num_zeros=num_zeros-1
                                                        if map1[i-1][j]==0:
                                                            map1[i-1][j]=k+1
                                                            num_zeros=num_zeros-1
                                                        if map1[i-1][j+1]==0:
                                                            map1[i-1][j+1]=k+1
                                                            num_zeros=num_zeros-1
                                                        if map1[i][j-1]==0:
                                                            map1[i][j-1]=k+1
                                                            num_zeros=num_zeros-1
                                                        if map1[i][j+1]==0:
                                                            map1[i][j+1]=k+1
                                                            num_zeros=num_zeros-1
                                                        if map1[i+1][j-1]==0:
                                                            map1[i+1][j-1]=k+1
                                                            num_zeros=num_zeros-1
                                                        if map1[i+1][j]==0:
                                                            map1[i+1][j]=k+1
                                                            num_zeros=num_zeros-1
                                                        if map1[i+1][j+1]==0:
                                                            map1[i+1][j+1]=k+1
                                                            num_zeros=num_zeros-1
                                                elif i-1>=0 and j-1>=0:
                                                        if map1[i-1][j-1]==0:
                                                                map1[i-1][j-1]=k+1
                                                                num_zeros=num_zeros-1
                                                elif i-1>=0:
                                                        if map1[i-1][j]==0:
                                                                map1[i-1][j]=k+1
                                                                num_zeros=num_zeros-1
                                                elif i-1>=0 and j+1<cols:
                                                        if map1[i-1][j+1]==0:
                                                                map1[i-1][j+1]=k+1
                                                                num_zeros=num_zeros-1
                                                elif j-1>=0:
                                                        if map1[i][j-1]==0:
                                                                map1[i][j-1]=k+1
                                                                num_zeros=num_zeros-1
                                                elif j+1<cols:
                                                        if map1[i][j+1]==0:
                                                                map1[i][j+1]=k+1
                                                                num_zeros=num_zeros-1
                                                elif i+1<rows and j-1>=0:
                                                        if map1[i+1][j-1]==0:
                                                                map1[i+1][j-1]=k+1
                                                                num_zeros=num_zeros-1
                                                elif i+1<rows:
                                                        if map1[i+1][j]==0:
                                                                map1[i+1][j]=k+1
                                                                num_zeros=num_zeros-1
                                                elif i+1<rows and j+1<cols:
                                                        if map1[i+1][j+1]==0:
                                                                map1[i+1][j+1]=k+1
                                                                num_zeros=num_zeros-1
                                
                        k=k+1

                print('K of brushfire') 
                print(k)

                '''
                best=np.amax(map1)
                #print(best)
                index=[]

                for k in range(4,rows-5):
                        for j in range(4,cols-5):
                                if map1[k][j]==best:
                                        inr=[k,j]
                                        index.append(inr)
                
                N=[]
                index=np.asarray(index)
                for i in range(index.shape[0]):
                        n = 0
                        for k in range(-4,4):
                                for j in range(-4,4):
                                        if map1[index[i][0]+k][index[i][1]+j]==-1:
                                                n=n+1
                        N.append(n)     

                max_value = max(N)
                max_index = N.index(max_value)          
                bestPoint=[index[max_index][0],index[max_index][1]]
                BestGazebo=[float(index[max_index][0]*self.res+self.xorg),float(index[max_index][1]*self.res+self.yorg)]
                
                print('Best point is ' )
                print(bestPoint)
                print('best point in Gazebo is')
                print(BestGazebo)
                bestPoint = np.array(bestPoint)
                BestGazebo = np.array(BestGazebo)
                '''

                return np.asarray(map1) #bestPoint, BestGazebo 

        # =================================================
        def  wavefront(self, map2, start, goal):
                rows = map2.shape[0]
                cols = map2.shape[1]
                goal_row=goal[0][0]
                print(goal_row)
                goal_col=goal[0][1]
                print(goal_col)
                start_row=start[0][0]
                start_col=start[0][1]
                map2[goal_row][goal_col]=2
                print('map2[goal_row][goal_col]')
                print(map2[goal_row][goal_col])
                print(map2)
                num_zeros = (map2 == 0).sum()
                k=2 
                while num_zeros>0:
                        for i in range(rows):
                                for j in range(cols):
                                        if map2[i][j]==k:
                                                
                                                if i-1>=0 and i+1<rows and j-1>=0 and j+1<cols:

                                                        if map2[i-1][j-1]==0:
                                                                map2[i-1][j-1]=k+1
                                                                num_zeros=num_zeros-1
                                                        if map2[i-1][j]==0:
                                                                map2[i-1][j]=k+1
                                                                num_zeros=num_zeros-1
                                                        if map2[i-1][j+1]==0:
                                                                map2[i-1][j+1]=k+1
                                                                num_zeros=num_zeros-1
                                                        if map2[i][j-1]==0:
                                                                map2[i][j-1]=k+1
                                                                num_zeros=num_zeros-1
                                                        if map2[i][j+1]==0:
                                                                map2[i][j+1]=k+1
                                                                num_zeros=num_zeros-1
                                                        if map2[i+1][j-1]==0:
                                                                map2[i+1][j-1]=k+1
                                                                num_zeros=num_zeros-1
                                                        if map2[i+1][j]==0:
                                                                map2[i+1][j]=k+1
                                                                num_zeros=num_zeros-1
                                                        if map2[i+1][j+1]==0:
                                                                map2[i+1][j+1]=k+1
                                                                num_zeros=num_zeros-1
                                                elif i-1>=0 and j-1>=0:
                                                        if map2[i-1][j-1]==0:
                                                                map2[i-1][j-1]=k+1
                                                                num_zeros=num_zeros-1
                                                elif i-1>=0:
                                                        if map2[i-1][j]==0:
                                                                map2[i-1][j]=k+1
                                                                num_zeros=num_zeros-1
                                                elif i-1>=0 and j+1<cols:
                                                        if map2[i-1][j+1]==0:
                                                                map2[i-1][j+1]=k+1
                                                                num_zeros=num_zeros-1
                                                elif j-1>=0:
                                                        if map2[i][j-1]==0:
                                                                map2[i][j-1]=k+1
                                                                num_zeros=num_zeros-1
                                                elif j+1<cols:
                                                        if map2[i][j+1]==0:
                                                                map2[i][j+1]=k+1
                                                                num_zeros=num_zeros-1
                                                elif i+1<=rows and j-1>=0:
                                                        if map2[i+1][j-1]==0:
                                                                map2[i+1][j-1]=k+1
                                                                num_zeros=num_zeros-1
                                                elif i+1<rows:
                                                        if map2[i+1][j]==0:
                                                                map2[i+1][j]=k+1
                                                                num_zeros=num_zeros-1
                                                elif i+1<rows and j+1<cols:
                                                        if map2[i+1][j+1]==0:
                                                                map2[i+1][j+1]=k+1
                                                                num_zeros=num_zeros-1
                        k=k+1
                traject=[]
                point = [start_row,start_col]
                traject.append(point)
                cr=start_row
                cc=start_col
                print('You are in WF')
                print(cr)
                print(cc)
                print('map2.shape')
                print(map2.shape)
                print('k')
                print(k)

                while map2[cr][cc] -2:
                        if map2[cr-1][cc]<map2[cr][cc] and map2[cr-1][cc]>1:
                                cr=cr-1
                                point=[cr,cc]
                                traject.append(point)
                        elif map2[cr+1][cc]<map2[cr][cc] and map2[cr+1][cc]>1:
                                cr=cr+1
                                point=[cr,cc]
                                traject.append(point)
                        elif map2[cr][cc-1]<map2[cr][cc] and map2[cr][cc-1]>1:
                                cc=cc-1
                                point=[cr,cc]
                                traject.append(point)
                        elif map2[cr][cc+1]<map2[cr][cc] and map2[cr-1][cc+1]>1:
                                cc=cc+1
                                point=[cr,cc]
                                traject.append(point)
                        elif map2[cr-1][cc-1]<map2[cr][cc] and map2[cr-1][cc-1]>1:
                                cr=cr-1
                                cc=cc-1
                                point=[cr,cc]
                                traject.append(point)
                        elif map2[cr-1][cc+1]<map2[cr][cc] and map2[cr-1][cc+1]>1:
                                cr=cr-1
                                cc=cc-1
                                point=[cr,cc]
                                traject.append(point)
                        elif map2[cr+1][cc-1]<map2[cr][cc] and map2[cr+1][cc-1]>1:
                                cr=cr+1
                                cc=cc-1
                                point=[cr,cc]
                                traject.append(point)
                        elif map2[cr+1][cc+1]<map2[cr][cc] and map2[cr+1][cc+1]>1:
                                cr=cr+1
                                cc=cc-1
                                point=[cr,cc]
                                traject.append(point)
                point=[goal_row,goal_col]
                traject.append(point)

                print('You Finished')

                traject=np.asarray(traject)
                print(traject)

                return map2,traject


        def ProcessProjectedMap(self):
                # get map
                # reshape the map to the given dimensions 
                rospy.sleep(1)
                data1 = np.reshape(self.dat, (self.wid ,self.heigh), order="F")
                data2 = np.asarray(data1)
                print('sahpe of map is:')
                print(data2.shape)

                StartPointWF=np.array([[int(math.floor((self.current_position[0]-self.xorg)/self.res)),int(math.floor((self.current_position[1]-self.yorg)/self.res))]])
                

                # Dilation 
                data3 = ndimage.grey_dilation(data2, footprint=np.ones((9,9)))
                #data3 = data2
                #print(type(data2))
                #print(data2.shape

                
                # Call brushfire 
                #map1 = self.brushfire(data3)
                print('size of map before brushfire: ')
                print(data3.shape)
                print('start point is:')
                print(StartPointWF[0][0])
                print(StartPointWF[0][1])

                WS = 20
                if (StartPointWF[0][0] <= WS or StartPointWF[0][1] <= WS):
                        map1 = self.brushfire(data3)
                else:
                        map1 = self.brushfire(data3)
                
                #map1 = self.brushfire(data3[StartPointWF[0]-WS : StartPointWF[0]+WS][StartPointWF[1]-WS : StartPointWF[1]+WS])
                print('size of map after brushfire: ')
                print(map1.shape)
                #[value_m, BestGazebo, map1] = self.brushfire(data3[StartPointWF[0]-100:StartPointWF[0]+100][StartPointWF[1]-100:StartPointWF[1]+100])

                #[value_m, BestGazebo] = self.FindBestPoint(map1)
                BestGazeboArray = self.FindBestPoint(map1)
                
                #GoalPoint = BestGazebo

                '''
                print(value_m.shape)
                data4 = np.reshape(self.dat, (self.wid ,self.heigh), order="F")
                data5 = np.asarray(data4)
                data6 = ndimage.grey_dilation(data5, footprint=np.ones((15,15)))
                GoalPoint = np.array([BestGazebo])
                [map2,traject]=self.wavefront(data6,StartPointWF,value_m)
                print('traject shape')
                print(traject.shape)
                traject=np.asarray(traject)
                rows_traj = traject.shape[0]
                cols_traj = traject.shape[1]
                map2[traject[0][0]][traject[0][1]]=10
                for i in range (1,rows_traj-2):
                        map2[traject[i][0]][traject[i][1]]=120
                map2[traject[rows_traj-1][0]][traject[rows_traj-1][1]]=130
                plt.imshow(map2)
                plt.show()

                trajectoryGazebo=np.zeros((rows_traj,2))

                for i in range (rows_traj):
                        trajectoryGazebo[i][0]=float(traject[i][0]**self.res+self.xorg)
                        trajectoryGazebo[i][1]=float(traject[i][1]**self.res+self.yorg)
                print(trajectoryGazebo)
                #print(value_m)
                #print(GoalPoint.shape)
                '''

                # Send request to /goto service 
                
                for i in range(BestGazeboArray.shape[0]):
                	print('got the point ... finding a path to point : ')
                	print(BestGazeboArray[i][0])
                	print(BestGazeboArray[i][1])
                	goto_request = GotoWaypointRequest()
                	goto_request.goal_state_x = round(BestGazeboArray[i][0], 1)
                	goto_request.goal_state_y = round(BestGazeboArray[i][1], 1)
                
                	goto_response = self.goto_serv_(goto_request)
                	
                	if(goto_response == 1):
                		break
                
                

                
                #data1 = np.reshape(data,(155,120), order="F")
                #data2 = np.asarray(data1)
                #data3 = ndimage.grey_dilation(data2, footprint=np.ones((7,7)))

                # wavefront algorithm
                #[value_map, traject] = self.wavefront(data3, np.array([self.current_position]), GoalPoint)
                #traject=np.asarray(traject)
                #print(traject)
                #rows_traj=traject.shape[0]
                #cols_traj=traject.shape[1]
                #value_map[traject[0][0]][traject[0][1]]=110

                #for i in range (1,rows_traj-2):
                #       value_map[traject[i][0]][traject[i][1]]=120

                #value_map[traject[rows_traj-1][0]][traject[rows_traj-1][1]]=130
                #plt.imshow(value_map)
                #plt.show()
                #rospy.spin()


#if __name__ == "__main__":
#       pm = ProcessMap()
#       pm.ProcessProjectedMap()
    #main()

