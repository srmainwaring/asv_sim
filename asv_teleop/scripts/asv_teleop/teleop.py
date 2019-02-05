# 
# Borrows from uuv_simulator/uuv_control_cascaded_pids/launch/joy_velocity.launch
#

from geometry_msgs.msg import Twist, Vector3
import os
import rospy
from sensor_msgs.msg import Joy

class Teleop:
  def __init__(self):
    # Publishers
    self.__output_pub = rospy.Publisher('output', Twist, queue_size=1)

    # Subscribers
    self.__joy_sub = rospy.Subscriber('joy', Joy, self.__joy_callback)

    # Parameters for axes mapping and gain
    self._axes = dict(
      x=3, y=2, z=1,
      yaw=0
    )

    self._axes_gain = dict(
      x=3, y=3, z=0.5,
      yaw=0.5
    )

    if rospy.has_param('~mapping'):
      mapping = rospy.get_param('~mapping')
      for tag in self._axes:
        if tag not in mapping:
          rospy.loginfo('Tag not found in axes mapping, tag={}'.format(tag))
        else:
          if 'axis' in mapping[tag]:
            self._axes[tag] = int(mapping[tag]['axis'])
          if 'gain' in mapping[tag]:
            self._axes_gain[tag] = float(mapping[tag]['gain'])

    # Parameter for deadzone
    self._deadzone = 0.5

  def __joy_callback(self, joy):
    # Debug Info
    rospy.loginfo('axes: {0}'.format(joy.axes))
    rospy.loginfo('buttons: {0}'.format(joy.buttons))

    # Linear velocities
    linear = Vector3(0, 0, 0)
    if self._axes['x'] > -1 and abs(joy.axes[self._axes['x']]) > self._deadzone:
      linear.x += self._axes_gain['x'] * joy.axes[self._axes['x']]

    if self._axes['y'] > -1 and abs(joy.axes[self._axes['y']]) > self._deadzone:
      linear.y += self._axes_gain['y'] * joy.axes[self._axes['y']]

    if self._axes['z'] > -1 and abs(joy.axes[self._axes['z']]) > self._deadzone:
      linear.z += self._axes_gain['z'] * joy.axes[self._axes['z']]

    # linear.x += self._axes_gain['x'] * joy.axes[self._axes['x']]
    # linear.y += self._axes_gain['y'] * joy.axes[self._axes['y']]
    # linear.z += self._axes_gain['z'] * joy.axes[self._axes['z']]

    # Angular velocities
    angular = Vector3(0, 0, 0)
    if self._axes['yaw'] > -1 and abs(joy.axes[self._axes['yaw']]) > self._deadzone:
        angular.z += self._axes_gain['yaw'] * joy.axes[self._axes['yaw']]

    # angular.z += self._axes_gain['yaw'] * joy.axes[self._axes['yaw']]

    cmd_vel = Twist()
    cmd_vel.linear = linear
    cmd_vel.angular = angular
    self.__output_pub.publish(cmd_vel)

def run_node():
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name, anonymous=True)
  rospy.loginfo('Starting [{0}] node'.format(node_name))

  teleop = Teleop()

  rospy.spin()
  rospy.loginfo('Shutting down [{0}] node',format(node_name))
