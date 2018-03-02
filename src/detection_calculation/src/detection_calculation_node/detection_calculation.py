#!/usr/bin/env python

import rospy
import yaml
import sys
from rospy import ROSException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from NewDetectionMessage.msg import NewDetectionMessage
from NewDetectionMessage.msg import Human
import math


global MyHumans
global init_robot_pose
global robot_pos_x, robot_pos_z, robot_pos_th

#RosLaunch Parameters
mission_number_ = rospy.get_param('~mission_number')
robot_number_ = rospy.get_param('~robot_number#')

#Config file dictinary
MyHumans = yaml.load(open('human.yaml'))
init_robot_pose = yaml.load(open('robot.yaml'))

robot_pos_x = init_robot_pose[str(mission_number_)][str(robot_number_)]['x']
robot_pos_z = init_robot_pose[str(mission_number_)][str(robot_number_)]['z']
robot_pos_th = init_robot_pose[str(mission_number_)][str(robot_number_)]['theta']


def process():
  rospy.init_node('detection_calculation_node', anonymous=True)
  rospy.Subscriber('robot4/odom', Odometry, Odometry_update)
  rospy.Subscriber('/robot4/camera/rgb/image_raw', Image, imageCallBack)

  rospy.spin()


def Odometry_update(data):
  #Getting x and z change for robot
  x = data.pose.pose.position.x
  z = data.pose.pose.position.z

  #Getting the Quaternion info
  xQ = data.pose.pose.orientation.x
  yQ = data.pose.pose.orientation.y
  zQ = data.pose.pose.orientation.z
  wQ = data.pose.pose.orientation.w

  # Converting quaternion to euler angle
  xE,yE,zE = quaternion_to_euler_angle(xQ,yQ,zQ,wQ)

  #Robot position constantly updated
  robot_pos_x = robot_pos_x + x
  robot_pos_z = robot_pos_z + z
  robot_pos_th = robot_pos_th + yE
  
  #Searching for humans
  find(robot_pos_x, robot_pos_z, robot_pos_th)


#Conversion Function 
def quaternion_to_euler_angle(w, x, y, z):
  ysqr = y * y

  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + ysqr)
  X = math.atan2(t0, t1)

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  Y = math.asin(t2)

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (ysqr + z * z)
  Z = math.atan2(t3, t4)
	
  return X, Y, Z  


def shift_points(RX,RZ,HX,HZ):
	HX = HX - RX
	HZ = HZ - RZ

	return 0,0,HX,HZ


def cartesian_to_polar_distance(x,z):
  return math.sqrt(x**2 + z**2)


#returning rad
def cartesian_to_polar_angle(x,z):
  return math.atan2(z/x)



def imageCallBack(data):
  print("ss")



def find(RoboPosX, RoboPosZ, RoboPosTh):
  for i in range(0,291):
    dist = math.sqrt( (RoboPosX - MyHumans[str(i)]['x'])**2 + (RoboPosZ - MyHumans[str(i)['z']])**2 )
    if dist <= 0.5:  #dof
    	rx,rz,hx,hz = shift_points(robot_pos_x,robot_pos_z, MyHumans[str(i)]['x'], MyHumans[str(i)]['z'])
    	human_degree = math.degree(cartesian_to_polar_angle(hx, hz))
    	robot_degree = math.degree(RoboPosTh)
    	FOV_degree = math.degree(init_robot_pose[str(mission_number_)][str(robot_number_)]['fov'])/2.0		#field of view divided by two, relative to robot
    	if (human_degree <= robot_degree + FOV_degree ) and (human_degree >= robot_degree - FOV_degree and (MyHumans[str(i)]['dclass'] != 2)):
    		print('check')





def main():
  process()


if __name__ == "__main__":
  main()