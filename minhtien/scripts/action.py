import time
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import numpy as np
def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    x=[1,0]
    y=[0,0]
    theta=[180,0]
    i=0
    while not rospy.is_shutdown():
	    goal = MoveBaseGoal()
	    goal.target_pose.header.frame_id = "map"
	    goal.target_pose.header.stamp = rospy.Time.now()
	    goal.target_pose.pose.position.x = x[i]
	    goal.target_pose.pose.position.y =y[i]
	    goc=math.sin(np.pi*-theta[i]/360)
	    goal.target_pose.pose.orientation.z = goc
	    goal.target_pose.pose.orientation.w = math.sqrt(abs(1-goc*goc))
	    client.send_goal(goal)
	    wait = client.wait_for_result()
	    time.sleep(2)
	    i=i+1
	    if (i>=len(x)):
	    	i=0;
	  
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        pass
