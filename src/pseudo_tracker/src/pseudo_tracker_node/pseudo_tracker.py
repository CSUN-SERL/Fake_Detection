import rospy
import yaml
import sys
from rospy import ROSException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from new_detection_messages.msg import CompiledMessage
from new_detection_messages.msg import Human
import math




def main():
	process()


if __name__ == "__main__":
	main()