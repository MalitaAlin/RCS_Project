#!/usr/bin/env python
## ---------------- ALIN
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #
from Area_definition_and_movement import *

## ---------------- DETECTING THE PRESENCE AND POSITION OF THE OBSTACLE
def detect_obstacle(self,data):
        thr1 = 0.6 # Treshold for Front collision
        thr2 = 0.6 # Treshold for Left and Right collision
        if (data.ranges[0]<thr1 or data.ranges[15]<thr2 or data.ranges[345]<thr2): # Checks if there are obstacles in front and 15 degrees left and right
              print (f"Range data at 0 deg:  {data.ranges[0]}" )
              print (f"Range data at 15 deg:  {data.ranges[15]}")
              print ( f"Range data at 345 deg: {data.ranges[345]}") 
              if (data.ranges[0]<thr1 and data.ranges[15]<thr2 and data.ranges[345]<thr2): #Large obstacle case
                 print ('---Large Obstacle Detected---')
                 self.obstacle['obstacle'] = add_obstacle(data.ranges[0], data.ranges[15], data.ranges[345],2)
                 return 2
              print ('---Small Obstacle Detected---')
              self.obstacle['obstacle']  = add_obstacle(data.ranges[0], data.ranges[15], data.ranges[345],1)
              return 1
        return 0 #case of no obstacle detected

## ---------------- ADDING OBSTACLE OBJECT TO CONTROLLER MEMORY
def add_obstacle(front_dist, left_dist, right_dist, size):
    #Obstacle types: { Small=>size=1, Large=>size=2}
    return [front_dist, left_dist, right_dist,size]

## ----------------AVOIDANCE STRATEGY
def avoid_obstacle(self): 
    # def avoid_obstacle(self,goal_x,goal_y):  previous version that would take goal into account 
    thr1 = 0.6 # Treshold 
    twist=Twist()
    angular_speed = 0.3
    linear_speed = 0.1

    if(self.obstacle['obstacle'][3] == 1): # small obstacle algorithm
      if(self.obstacle['obstacle'][1] > self.obstacle['obstacle'][2] ):    
        twist.angular.z=angular_speed #rotate to the left if 
      else:
       twist.angular.z=-angular_speed 

      self.velocity_publisher.publish(twist)
      rospy.sleep(4) # sleep after angular twist
      twist.angular.z=0.0
      twist.linear.x=linear_speed 
      self.velocity_publisher.publish(twist)
      rospy.sleep(2)  # sleep after short linear movement
      twist.linear.x=0.0 
      self.velocity_publisher.publish(twist)
      print ('---Avoiding Small Object---')

    elif(self.obstacle['obstacle'][3] == 2):  # large obstacle algorithm
   
     twist.linear.x=-linear_speed 
     self.velocity_publisher.publish(twist)
     rospy.sleep(3)# sleep after moving backwards
      #Large object evasion
     if(self.obstacle['obstacle'][1] > thr1 and self.obstacle['obstacle'][2]  ):
         twist.angular.z=angular_speed
     else:
       twist.angular.z=-angular_speed 

     twist.linear.x=0.0
     self.velocity_publisher.publish(twist)
     rospy.sleep(4)# sleep after angular twist
     twist.angular.z=0.0
     twist.linear.x=linear_speed 
     self.velocity_publisher.publish(twist)
     rospy.sleep(3)  # sleep after short linear movement

     twist.linear.x=0.0 
     self.velocity_publisher.publish(twist)
     print ('---Avoiding Large Object---')

    self.obstacle_detected = 0 #reset the obstacle detection
 

    
       
       