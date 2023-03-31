#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, Twist, Point, Pose, Quaternion, Vector3, PoseWithCovarianceStamped
from tf2_ros import  StaticTransformBroadcaster 
import numpy as np
import math
from nav_msgs.msg import Odometry
import tf
global vantocx,vantocy,vantocz,rota,ts,vantocz1,vantocz2,vantocz22,x,y,sp_yaw,ki,dem,theta,flat
flat=0
theta=0
vantocx=0
vantocy=0
vantocz=0
vantocz1=0
vantocz2=0
vantocz22=0
x=0
y=0
dem=0
ki=0
sp_yaw=0
ts=TransformStamped()
ts.header.frame_id='/odom'
ts.child_frame_id='base_footprint'
rota=[0,0,0,1]

def callback1(A):
    global ts,vantocx,vantocy,vantocz,rota,vantocz1,vantocz2,x,y,z,sp_yaw,dem
    vantocz1=A.angular.x/2
    if vantocz1>90:
        vantocz1=vantocz1-180
    elif vantocz1<-90:
        vantocz1=vantocz1+180
    vantocz=math.sin(np.pi*vantocz1/180)
    vantocx=A.linear.x/100
    if vantocz>1:
        vantocz=vantocz-2
    elif vantocz<-1:
        vantocz=vantocz+2
    x= ts.transform.translation.x
    y= ts.transform.translation.y
    print(vantocz)
def callback2(A):
    global theta,flat
    theta1=A.pose.pose.orientation.z
    theta=180*2*math.asin(theta1)/3.14
    flat=1
def callback(A):
    global ts,vantocx,vantocy,vantocz,rota,vantocz1,vantocz2,vantocz22,x,y,z,ki,sp_yaw,flat,theta
    ts.header.frame_id='/odom'
    ts.child_frame_id='base_footprint'

    #vantocz1=vantocz1+360*A.angular.z/3.14
    #if vantocz1>90:
    #    vantocz1=vantocz1-180
    #elif vantocz1<-90:
    #    vantocz1=vantocz1+180
    #vantocz=math.sin(np.pi*vantocz1/180)

    #ts.transform.translation.x=x+A.linear.x*np.cos(2*math.atan(vantocz/(math.sqrt(abs(1-vantocz*vantocz))+0.000000000000000000000000000001)))
    #ts.transform.translation.y=y+A.linear.x*np.sin(2*math.atan(vantocz/(math.sqrt(abs(1-vantocz*vantocz))+0.000000000000000000000000000001)))
    #print(ts.transform.translation.x)
    #vantocx=A.linear.x/5
    #vantocy=vantocy+180*(A.angular.y/25)/np.pi
    #x= ts.transform.translation.x
    #y= ts.transform.translation.y
    #z= ts.transform.translation.z

   
    ts.transform.translation.z=A.linear.z/25
    rotaa=[theta,flat,A.angular.z,0]
    #rota=np.linalg.norm(rotaa)
    #rota=rotaa/rota
    #hh=transform_broadcaster.TransformBroadcaster()
    #hh.sendTransform([ts])
    odomm=Odometry()
    odomm.header.frame_id='odom'
    odomm.child_frame_id='base_footprint'
    #odom_quat = tf.transformations.quaternion_from_euler(0, 0, 2*math.atan(vantocz/(math.sqrt(abs(1-vantocz*vantocz)))))
    odomm.pose.pose=Pose(Point(x,y,0),Quaternion(*rotaa))
    print(theta)
    odomm.twist.twist=Twist(Vector3(A.linear.x, A.linear.y/5, 0), Vector3(theta,flat, A.angular.z))
    odom_pub.publish(odomm)
    flat=0
if __name__=="__main__":
    try:
        rospy.init_node("toado")
        odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        rospy.Subscriber('/cmd_vel',Twist,callback)
        rospy.Subscriber('/nhan_uart',Twist,callback1)
        rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,callback2)
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            #rospy.INFO(ts.header.frame_id)
            rate.sleep()
            ts.header.stamp=rospy.Time.now()
            #hhh=StaticTransformBroadcaster()
            ts.transform.translation.x=x+vantocx*np.cos(2*math.atan(vantocz/(math.sqrt(abs(1-vantocz*vantocz))+0.000000000000000000000000000001)))
            ts.transform.translation.y=y+vantocx*np.sin(2*math.atan(vantocz/(math.sqrt(abs(1-vantocz*vantocz))+0.000000000000000000000000000001)))
            hhh=tf.TransformBroadcaster()
            rota=[0,0,vantocz,math.sqrt(abs(1-vantocz*vantocz))]
            translation=(ts.transform.translation.x,ts.transform.translation.y,ts.transform.translation.z)
            hhh.sendTransform(translation,rota,ts.header.stamp,ts.child_frame_id,ts.header.frame_id)
    except rospy.ROSInterruptException:
        pass
