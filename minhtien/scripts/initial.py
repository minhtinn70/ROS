#!/usr/bin/env python3
import tf2_ros
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf
global translate,rota,ts
rota = tf.transformations.quaternion_from_euler(0,0,0)
ts=TransformStamped()
ts.header.frame_id="map"
ts.child_frame_id="odom"
ts.transform.rotation.x=0
ts.transform.rotation.y=0
ts.transform.rotation.z=0
ts.transform.rotation.w=1
translate=(0,0,0)
def callback(A):
    global translate,rota,ts
    #A=PoseWithCovarianceStamped()
    translate=(A.pose.pose.position.x,A.pose.pose.position.y,A.pose.pose.position.z)
    #rota=(A.pose.pose.orientation.x,A.pose.pose.orientation.y,A.pose.pose.orientation.z)
    rota = (A.pose.pose.orientation.x,A.pose.pose.orientation.y,A.pose.pose.orientation.z,A.pose.pose.orientation.w)
    frame_child='odom'
    rospy.loginfo(A.header.stamp)
    frame_id=A.header.frame_id

    ts.header.frame_id="map"
    ts.child_frame_id="odom"
    ts.transform.translation.x=A.pose.pose.position.x
    ts.transform.translation.y=A.pose.pose.position.y
    ts.transform.translation.z=A.pose.pose.position.z

    ts.transform.rotation.x=A.pose.pose.orientation.x
    ts.transform.rotation.y=A.pose.pose.orientation.y
    ts.transform.rotation.z=A.pose.pose.orientation.z
    ts.transform.rotation.w=A.pose.pose.orientation.w
    ts.header.stamp=rospy.Time.now()
    ta=tf2_ros.StaticTransformBroadcaster()
    ta.sendTransform([ts])
if __name__=="__main__":
    try:
        rospy.init_node("initial")
        ros=rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,callback)
        #listener = tf.TransformListener()
        #listener.waitForTransform('base_footprint', 'odom', rospy.Time.now(), rospy.Duration(1.0))
        rate=rospy.Rate(100)
        #while not rospy.is_shutdown():
        ts.header.frame_id="map"
        ts.child_frame_id="odom"
        ts.header.stamp=rospy.Time.now()
        ta=tf2_ros.StaticTransformBroadcaster()
        ta.sendTransform([ts])
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
