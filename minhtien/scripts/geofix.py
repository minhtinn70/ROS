#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, Twist, Point, Pose, Quaternion, Vector3
import numpy as np
import math
from nav_msgs.msg import Odometry, Path
import tf
from move_base_msgs.msg import MoveBaseActionGoal 
global vantoc,rota,ts,x,y,xs,ys,orientationZ,orientationW, xgoal,ygoal,zgoal,wgoal
vantoc=0
x=0
y=0
xs=0
ys=0
orientationZ=0
orientationW=1
xgoal=0
ygoal=0
zgoal=0
wgoal=0
vantoc=math.sqrt((x-xs)*(x-xs)+(y-ys)*(y-ys))
ts=TransformStamped()
ts.header.frame_id='/odom'
ts.child_frame_id='base_footprint'
rota=[0,0,0,1]
T=MoveBaseActionGoal()
T.goal.target_pose.pose
def goal(A):
    xgoal=A.goal.target_pose.pose.position.x
    ygoal=A.goal.target_pose.pose.position.y
    zgoal=A.goal.target_pose.pose.orientation.z
    wgoal=A.goal.target_pose.pose.orientation.w
def callback(A):
    
    global vantoc,rota,ts,x,y,xs,ys,orientationZ,orientationW
    #print(len(A.poses))
    x=A.poses[len(A.poses)-1].pose.position.x
    y=A.poses[len(A.poses)-1].pose.position.y

    vantoc=math.sqrt((x- ts.transform.translation.x)*(x-ts.transform.translation.x)+(y-ts.transform.translation.y)*(y-ts.transform.translation.y))/5
    
    xs=A.poses[len(A.poses)-1].pose.position.x
    ys=A.poses[len(A.poses)-1].pose.position.y

    orientationZ=A.poses[len(A.poses)-1].pose.orientation.z
    orientationW=A.poses[len(A.poses)-1].pose.orientation.w
    
    ts.transform.translation.x=ts.transform.translation.x+vantoc*np.cos(2*math.atan(orientationZ/(math.sqrt(abs(1-orientationZ*orientationZ))+0.000000000000000000000000000001)))
    ts.transform.translation.y=ts.transform.translation.y+vantoc*np.sin(2*math.atan(orientationZ/(math.sqrt(abs(1-orientationZ*orientationZ))+0.000000000000000000000000000001)))
    #print(orientationW)
if __name__=="__main__":
    try:
        rospy.init_node("toadoo")
        #odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        rospy.Subscriber('/move_base/DWAPlannerROS/local_plan',Path,callback)
        rospy.Subscriber('/move_base/goal',MoveBaseActionGoal,goal)
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            ts.header.stamp=rospy.Time.now()
            hhh=tf.TransformBroadcaster()
            rota=[0,0,orientationZ,orientationW]
            translation=(ts.transform.translation.x,ts.transform.translation.y,ts.transform.translation.z)
            hhh.sendTransform(translation,rota,ts.header.stamp,ts.child_frame_id,ts.header.frame_id)
    except rospy.ROSInterruptException:
        pass