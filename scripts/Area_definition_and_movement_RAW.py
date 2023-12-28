#!/usr/bin/env python

## ----------------- IMPORTS
from socket import timeout
import time
import signal
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import atan2, pi, sqrt, sin, cos
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_multiply, quaternion_conjugate, quaternion_inverse
import numpy as np

## ----------------- PATROLER CLASS
class TurtleBotPatroler:
    ## ----------------- INITIALIZATION
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
        
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.update_scan)
        
        self.pose = None
        self.scan_data = None
        
        self.rate = rospy.Rate(10) # Creates a Rate object with a frequency of 10 Hz
        self.areas = {}   # Initialize self.areas as an empty dictionary

    ## ----------------- UPDATING POSITION
    def update_pose(self, data):
        self.pose = data.pose.pose
    
    ## ----------------- UPDATING SCANNER 
    def update_scan(self, data):
        self.scan_data = data.ranges
    
    ## ----------------- CALCULATING POSITION BETWEEN TWO POINTS
    def get_distance(self, goal_x, goal_y):
        
        if self.pose is None:
            return None
        
        print(f"current x is: {self.pose.position.x}")
        print(f"current y is: {self.pose.position.y}")
        x = self.pose.position.x - goal_x
        y = self.pose.position.y - goal_y

        return sqrt(x*x + y*y)        


    ##---------------- CALCULATING GOAL ANGLE
    def goal_angle_is(self,goal_x,goal_y):
        
        if self.pose is None:
            return None
        
        goal_angle = atan2(goal_y - self.pose.position.y, goal_x - self.pose.position.x)
        
        print(f"Calculated Goal Angle: {goal_angle}")        
        return goal_angle

    ## ----------------- CALCULATING ANGLE DIFFERENCE BETWEEN desired position and current one
    def get_angle_difference(self,goal_angle):
        
        if self.pose is None:
            return None
        
        current_quaternion = [ self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w ]

        # Convert the angle to quaternion
        goal_quaternion = [ 0.0, 0.0, sin(goal_angle / 2), cos(goal_angle / 2) ]

        # Convert both quaternions to unit quaternions
        current_quaternion = current_quaternion / np.linalg.norm(current_quaternion)
        goal_quaternion = goal_quaternion / np.linalg.norm(goal_quaternion)

        # Calculate the quaternion difference
        quaternion_diff = quaternion_multiply(goal_quaternion, quaternion_inverse(current_quaternion))

        # Convert the quaternion difference to Euler angles
        _, _, angle_difference = euler_from_quaternion(quaternion_diff)

        print(f"Calculated Angle Difference: {angle_difference}")
        return angle_difference
    
    ## ----------------- STOPPING THE ROBOT
    def stop_robot(self):

        twist = Twist()
        twist.linear.x = 0.0  # linear speed
        twist.angular.z = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        self.velocity_publisher.publish(twist)

        print(f"Robot stopped")

    ## ----------------- TIMEOUT check
    def timeout(self,start_time):
        
        max_time=120 # 120s
        current_time = time.time()
        elapsed_time = current_time - start_time

        if elapsed_time > max_time:
            print(f"Skipping the goal due to timeout.")
            return 1
        
        return 0
    
    ##---------- ALIGNING with goal angle
    def aligning(self,goal_angle,twist,angle_difference,start_time):

        # Stopping the robot
        self.stop_robot()

        # allowed error
        a_error=0.0001

        #speed
        speed=0.09
         
        while abs(angle_difference) > a_error:
            
            angle_difference=self.get_angle_difference(goal_angle) # recalculating angle_difference

            if angle_difference > 0:
                angular_speed = speed
            else: 
                angular_speed= (-1)*speed
            
            if angle_difference < a_error and angle_difference > 0: angular_speed = 0.0
            if angle_difference > (-1)*a_error and angle_difference < 0: angular_speed = 0.0
                
            twist.angular.z = angular_speed # modifies the twist object
            self.velocity_publisher.publish(twist) # publishes through twist cmd to robot
            self.rate.sleep() # ensures that a 10 Hz frequency is maintained

            # if rotating longer than a minute, exit loop
            to=self.timeout(start_time)
            if(to==1):
                print(f"Goal Angle: {goal_angle}")
                self.stop_robot()    
                break

        print(f"Goal Angle: {goal_angle}")

        #stop robot
        self.stop_robot()    

    ##------ TRAVERSE DISTANCE
    def traversing(self,goal_x,goal_y,twist,distance,start_time):
            
        # stopping the robot
        self.stop_robot()

        # allowed error
        t_error=0.3 # 30 cm

        # speed
        speed=0.1

        while distance > t_error: 
            
            # linear speed gets modified
            twist.linear.x = speed
            self.velocity_publisher.publish(twist)

            # recalculating distance
            distance = self.get_distance(goal_x, goal_y)

            # ensuring 10Hz
            self.rate.sleep()

            # if too far go to next goal
            if(distance > 7):
                print(f"Goal x: {goal_x}, Goal y: {goal_y}") 
                self.stop_robot()
                print(f"Distance is to great, skip goal")
                break

            # if moving longer than a set time, exit loop
            to=self.timeout(start_time)
            if(to==1):
                print(f"Goal x: {goal_x}, Goal y: {goal_y}") 
                self.stop_robot()    
                break
        
        print(f"Goal x: {goal_x}, Goal y: {goal_y}")        

        # Stopping the robot
        self.stop_robot()

    ## ----------------- MOVING TO GOAL
    def move_to_goal(self, goal_x, goal_y):

        
        # declaring twist to send msg
        twist = Twist()

        if self.pose is None:
            print(f"Pose is not updated yet.")
            return
        
        # starting measuring time
        start_time=time.time()

        # Calculating Goal angle
        goal_angle=self.goal_angle_is(goal_x,goal_y)  

        # aligning with desired position through rotation (angular)  

        angle_difference=self.get_angle_difference(goal_angle) 
        self.aligning(goal_angle,twist,angle_difference,start_time)

        # traversing distance (linear)

        distance = self.get_distance(goal_x, goal_y)
        self.traversing(goal_x,goal_y,twist,distance,start_time)
            
        

    ## ----------------- AREA to patrol DEFINITION
    def define_area(self, area_name, center,length):
        self.areas[area_name] = [center, length]

    ## ----------------- BOUNDARY points of area DEFINITION    
    def define_boundary(self, area_name):
        if area_name not in self.areas:
            print(f"Area '{area_name}' not found.")
            return
        
        boundary_points = []
        center = self.areas[area_name][0]  # Accessing the first two elements (index 0 and 1) for the center (x, y)
        length = self.areas[area_name][1]  # Accessing the third element (index 2) for the length/distance value
        
        boundary_points.append([center[0]+length/2,center[1]-length/2])
        boundary_points.append([center[0]-length/2,center[1]-length/2])
        boundary_points.append([center[0]-length/2,center[1]+length/2])
        boundary_points.append([center[0]+length/2,center[1]+length/2])
        boundary_points.append([center[0],center[1]])

        return boundary_points
    
    ## ----------------- PATROL IMPLEMENTATION
    def patrol_area(self, area_name):
        if area_name not in self.areas:
            print(f"Area '{area_name}' not found.")
            return       
       
        boundary_points=self.define_boundary(area_name)
        for point in boundary_points:
            print(f"Patrolling near: {point}")
            self.move_to_goal(point[0], point[1])
            self.rate.sleep()

##--------- SIGINT HANDLER
def sigint_handler_wrapper(patroller):
    def sigint_handler(signum, frame):
        print("Ctrl + C detected. Stopping the robot.")
        if patroller:
            patroller.stop_robot()
        rospy.signal_shutdown('Ctrl + C detected')
    return sigint_handler
    
##---------- USER INPUT AREA   
def define_area_console_input(patroller):
    area_name = input("Enter area name: ")
    center_x = float(input("Enter center X coordinate: "))
    center_y = float(input("Enter center Y coordinate: "))
    radius = float(input("Enter radius: "))
    patroller.define_area(area_name, (center_x, center_y), radius) 
    
##---------- USER INPUT AREA       
def what_area_to_patrol(patroller):
    area_name = input("What area should I patrol?")
    patroller.patrol_area(area_name)
    
## ----------------- MAIN
def main():
    try:
        patroller = TurtleBotPatroler()
        rospy.sleep(1)
        
        
        # Area Definition
        # patroller.define_area('Area1', (-3, 2.5), 2)  # Define your areas here
        # patroller.define_area('Area2', (-6.5, -1.5), 2)  # Define your areas here
        define_area_console_input(patroller)
              
        # Register the signal handler for Ctrl + C
        signal.signal(signal.SIGINT, sigint_handler_wrapper(patroller))

        what_area_to_patrol(patroller)
      
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

## --------
if __name__ == '__main__':
   main()