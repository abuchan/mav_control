#!/usr/bin/python

# This node controls a MAVLINK enabled vehicle using a joystick. It does this
# by putting the vechicle into an OFFBOARD armed state, and publishing directly
# to the actuator_controls topic (RPYT commands). By default it uses the first
# 4 axis of a joystick, but if using a DroidPad 
# (https://www.digitalsquid.co.uk/droidpad/) interface, you can select the
# correct axes by passing 'droidpad' as an argument to this script

import rospy
import sys

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import *

class JoystickToMAVROS():
  def __init__(self, droidpad=False):
    
    self.droidpad = droidpad

    rospy.init_node('joystick_to_mavros')
    
    self.set_mode_proxy = rospy.ServiceProxy('mavros/set_mode',SetMode)
    self.arm_proxy = rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
   
    self.actuator_controls_pub = rospy.Publisher(
      'mavros/actuator_control',
      ActuatorControl,
      queue_size = 1,
    )
    
    self.js_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
    
    self.state_sub = rospy.Subscriber(
      'mavros/state', 
      State, 
      self.state_callback,
      queue_size = 1,
    )

    self.state = State()
    
    self.informed = False

    self.actuator_controls = None

    self.rate = rospy.Rate(20)

  def run(self):
    while (not rospy.is_shutdown()):
      
      if self.state.mode != 'OFFBOARD':
        #print 'Setting Offboard mode'
        try:
          self.set_mode_proxy(0,'OFFBOARD')
        except rospy.ServiceException:
          #print 'Waiting for set_mode service'
          pass

      elif not self.state.armed:
        #print 'Offboard mode set, arming'
        try:
          self.arm_proxy(True)
        except rospy.ServiceException:
          # print 'Waiting for arming service'
          pass
      
      elif not self.informed:
        print 'Offboard mode armed, ready to command'
        self.informed = True
        
      self.publish_actuator_controls(self.actuator_controls)

      self.rate.sleep()

  def state_callback(self, msg):
    self.state = msg

  def joy_callback(self, msg):
    if self.droidpad:
      r,p,y,t = msg.axes[-4:]
    else:
      r,p,y,t = msg.axes[:4]
    self.actuator_controls = [r,p,y,t] 
  
  def publish_actuator_controls(self,controls=None):
    if controls is None:
      controls = []
    if type(controls) is not list:
      controls = list(controls)
    ac = ActuatorControl()
    ac.header.stamp = rospy.Time.now()
    ac.controls = controls + (8-len(controls)) * [0.0]
    self.actuator_controls_pub.publish(ac)
    
if __name__ == '__main__':
  droidpad = len(sys.argv) > 1 and sys.argv[1] == 'droidpad'
  jtm = JoystickToMAVROS(droidpad)
  jtm.run()

