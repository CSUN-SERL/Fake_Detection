#!/usr/bin/env python

import rospy
import yaml
import sys
from rospy import ROSException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose


global MyHumans
global robot_pose

def loadInfo():
  MyHumans = yaml.load(open('human.yaml'))
  #robot_pose = yaml.load(open('robot.yaml'))


def process():
  rospy.init_node('detection_calculation_node', anonymous=True)
  rospy.Subscriber('robot1/odom', Odometry, Odometry_update)
  rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, imageCallBack)

  rospy.spin()

def Odometry_update(data):
  x = msg.pose.pose.position.x
  y = msg.pose.pose.position.y
  #robot_pose['1']['1']['x'] = x
  #robot_pose['1']['1']['y'] = y
  find()


def imageCallBack(data):
  print("ss")


def find():
  for i in range(0,291):
    dist = sqrt( (robot_pose['1']['1']['x'] - MyHumans[str(i)]['x'])**2 + (robot_pose['1']['1']['y'] - MyHumans[str(i)['y']])**2 )
    if dist <= 10:
      print("Correct")

def main():
  loadInfo()
  process()


if __name__ == "__main__":
  main()