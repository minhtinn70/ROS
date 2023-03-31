#!/usr/bin/python3
import time
import serial
import json
import rospy
from geometry_msgs.msg import Twist
if __name__=="__main__":
    ser = serial.Serial(
        port = '/dev/ttyACM0',
        baudrate = 9600,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 0.1
    )
    rospy.init_node("rasdius")
    odom_pub = rospy.Publisher("nhan_uart", Twist, queue_size=1)
    print("Raspberry's receiving : ")
    try:
        while not rospy.is_shutdown():
            try:
                data=None
                s = ser.readline()
                #time.sleep(0.1)		
                #data_left = ser.inWaiting()
                #time.sleep(0.03)
                #s += ser.read(data_left)
                data = s.decode()			# decode s
                data = data.rstrip()			# cut "\r\n" at last of string
                print(data)
                data=eval(str(data))
                #data= '{"vantocx": 0.5, "vantocy": 0.0, "x": 1.7500000000000009, "y": 0.0, "th": 0.0, "dth": 0.0}'
                data=json.dumps(data)
                print(data)
                data=json.loads(data)
                #goc=data['th']
                odoom=Twist()
            #except SyntaxError:
            except (UnicodeDecodeError,SyntaxError):
                pass
            if data is not None:
                try:
                    odoom.angular.x=data["th"]
                    odoom.linear.x=data["vantocx"]
                    print(odoom.angular.x)
                    odom_pub.publish(odoom)
                except (TypeError, KeyError):
                    pass	
    except rospy.ROSInterruptException:
        pass
