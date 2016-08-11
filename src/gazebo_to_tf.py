#!/usr/bin/python

# This node will take the Model pose info from the /gazebo/model_states topic
# and publish it to the ROS standard TF topic, with all poses relative to the
# 'world' frame

import rospy
import tf
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Pose

class GazeboToTF():
  def __init__(self):
    rospy.init_node('gazebo_to_tf')
    self.ms_sub = rospy.Subscriber(
      '/gazebo/model_states', ModelStates, self.ms_callback, queue_size=1)
    self.tfb = tf.TransformBroadcaster()

  def run(self):
    while not rospy.is_shutdown():
      rospy.sleep(0.1)

  def ms_callback(self,msg):
    for n,p in zip(msg.name, msg.pose):
      T, O = p.position, p.orientation
      self.tfb.sendTransform(
        (T.x, T.y, T.z), (O.x, O.y, O.z, O.w), rospy.Time.now(), n, 'world')

if __name__ == '__main__':
  gtf = GazeboToTF()
  gtf.run()
  
