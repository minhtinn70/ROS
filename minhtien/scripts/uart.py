#!/usr/bin/python3
import json
from turtle import delay
import serial
import time
from geometry_msgs.msg import Twist
import rospy
from nav_msgs.msg import Odometry
global data,ser,key
key=0
data={
    "x": 0,
    "dth": 0,
    "f":0,
    "th":0,
   }

def callback(A):
    global data,ser,key
    data["dth"]=round(A.pose.pose.orientation.z,2)
    data["th"]=round(A.pose.pose.orientation.x,2)
    data["f"]=round(A.pose.pose.orientation.y,2)
    data["x"]=round(A.twist.twist.linear.x,2)
    print(data)

if __name__=="__main__":
    try:
        ser=serial.Serial (port="/dev/ttyACM0",baudrate=9600,timeout=0.5)
        rospy.init_node("thanhtu")
        rospy.Subscriber('odom',Odometry,callback)
        while not rospy.is_shutdown():
            time.sleep(0.1)
            data1=str.encode(str(data))
            ser.write(data1)
            ser.flush()
    except rospy.ROSInterruptException:
        pass

