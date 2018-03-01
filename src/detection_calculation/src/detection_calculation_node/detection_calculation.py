#!/usr/bin/env python

import rospy
import yaml
import sys
from rospy import ROSException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import math


global MyHumans
global robot_pose
MyHumans = yaml.load(open('human.yaml'))
robot_pose = yaml.load(open('robot.yaml'))

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
  robot_pose['mission1']['1']['x'] = robot_pose['mission1']['1']['x'] + x
  robot_pose['mission1']['1']['y'] = robot_pose['mission1']['1']['y'] + y
  robot_pose['mission1']['1']['theta'] = robot_pose['mission1']['1']['theta'] + yE
  
  #Searching for humans
  find()

#Conversion Function 
def quaternion_to_euler_angle(w, x, y, z):
  ysqr = y * y

  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + ysqr)
  X = math.degrees(math.atan2(t0, t1))

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  Y = math.degrees(math.asin(t2))

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (ysqr + z * z)
  Z = math.degrees(math.atan2(t3, t4))
	
	return X, Y, Z  

def cartesian_to_polar_distance(x,z):
  return math.sqrt(x**2 + z**2)

def cartesian_to_polar_angle(x,z):
  return math.degrees(math.atan(z/x))


def imageCallBack(data):
  print("ss")


def find():
  for i in range(0,291):
    dist = math.sqrt( (robot_pose['1']['1']['x'] - MyHumans[str(i)]['x'])**2 + (robot_pose['1']['1']['y'] - MyHumans[str(i)['y']])**2 )
    if dist <= 0.5:  #dof
      print("Correct")

def main():
  process()


if __name__ == "__main__":
  main()