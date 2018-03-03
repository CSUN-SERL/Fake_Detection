import rospy
import yaml
import sys
from rospy import ROSException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from new_detection_msgs.msg import CompiledMessage
from new_detection_msgs.msg import Human
import math

human_tracked = {}


def process():
  rospy.init_node('pseudo_trcker_node', anonymous=True)
  pub = rospy.Publisher('sarwai_detection/tracker_msgs', CompiledMessage, queue_size=100)
  rospy.Subscriber('sarwai_detection/custom_msgs_info', new_detection_msgs, getInfo)

  rospy.spin()


def getInfo(data):
	for i in range(0, len(data.humans)):
		if data.humans[i].id not in dict.keys():
			human_tracked[data.humans[i].id] = 1


def main():
	process()


if __name__ == "__main__":
	main()