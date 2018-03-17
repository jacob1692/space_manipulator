#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ModelStates


def Vector3subs(Vector2, Vector1):
	Vectorp=Vector3()
	Vectorp.x=Vector2.x-Vector1.x
	Vectorp.y=Vector2.y-Vector1.y
	Vectorp.z=Vector2.z-Vector1.z
	return Vectorp

def Twistsub(data): #Vector Substraction
	Twistp=Twist() 
	#Twistp is the result of the substraction between the twist of the target index [1] and the chaser index [0]
	Twistp.linear=Vector3subs(data.twist[1].linear,data.twist[0].linear)
	Twistp.angular=Vector3subs(data.twist[1].angular,data.twist[0].angular)
	return Twistp
	
def callback(data):	
	#	rospy.loginfo()
	if not rospy.is_shutdown() and pub.get_num_connections() >= 0 :
		if len(data.name)==2:
			pub.publish(Twistsub(data))

def listener():
    rospy.init_node('rel_vel_estimator', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    rospy.spin()
def publisher():  
	#Register a new topic
	global pub
	pub= rospy.Publisher("/relative_twist",Twist, queue_size=1000)
	
if __name__ == '__main__':
	#initialization of variables
	try:
		publisher()
	except rospy.ROSInterruptException: 
		pass
	listener()
