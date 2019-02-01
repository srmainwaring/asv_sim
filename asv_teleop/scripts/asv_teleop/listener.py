import os
import rospy
from sensor_msgs.msg import Joy

def callback(joy):
  rospy.loginfo('axes: {0}'.format(joy.axes))
  rospy.loginfo('buttons: {0}'.format(joy.buttons))

def run_node():
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name, anonymous=True)
  rospy.loginfo('Starting [{0}] node'.format(node_name))

  rospy.Subscriber('/rexrov/joy', Joy, callback)

  rospy.spin()
  rospy.loginfo('Shutting down [{0}] node',format(node_name))
