## Autonomous-Navigation-and-Exploration using Turtlebot

This  project  report  deals  with  work  done for  implementing  and  testing  different  highlevel  con-trollers  for  path  planning  algorithms  in  a  simulated  en-vironment. The work is done in a simulated environmentand  the  task  of  autonomous  navigation  in  an  unknown environment  is  projected.  The  project  is  divided  intotwo  parts  ,  the  first  step  is  to  navigate  in  an  unknown environment  for  certain  time  to  create  a  map  of  theplaced  system.  The  second  step  is  to  return  back  to  the home position from any point the exploration is stopped.

# Project flow

Exploration is carried out as shown in the figure below. Evidence grid is created to store the probability of  the  corresponding  region  in  any  space.  It  can fuse  information  from  different  types  of  sensors. Cells  are  initialized  at  a  prior  probability  withrough estimate of overall probability at any given location.  Once  evidence  grid  there,  for  each  cell occupancy probability is compared with the initialprobability  and  they  are  classified.   

<img src="https://github.com/bigmb/Autonomous-Navigation-and-Exploration/blob/master/1.png" width="450">

# Explored Regions

This is exploration carried out for around 50 min in willow garage map.

<img src="https://github.com/bigmb/Autonomous-Navigation-and-Exploration/blob/master/all.png" width="450">

For returning home, this is the path taken.

<img src="https://github.com/bigmb/Autonomous-Navigation-and-Exploration/blob/master/ex.png" width="450">


# Outcome

The robot was able to successfully explore the environment.

# Blog 
```
Coming Up
```
