#!/usr/bin/env python

#THIS SCRIPT IS TO BROADCAST THE TOTAL FORCE MEASURED BY THE CONTACT SENSOR REFERENCED IN THE WORLD REFERENCE FRAME


import sys
#load the package containing the message for penetration depths
#import roslib
#roslib.load_manifest('active_debris_control')
import rospy

from std_msgs.msg import String
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Vector3
#from std_msgs.msg import Float64
from active_debris_control.msg import StampedDepths

def lowpass(a):
	global wrench0
	global wrench
	f=Vector3()
	t=Vector3()
	f.x=(1-a)*(wrench.force.x)+(a)*(wrench0.force.x)
	f.y=(1-a)*(wrench.force.y)+(a)*(wrench0.force.y)
	f.z=(1-a)*(wrench.force.z)+(a)*(wrench0.force.z)
	t.x=(1-a)*(wrench.torque.x)+(a)*(wrench0.torque.x)
	t.y=(1-a)*(wrench.torque.y)+(a)*(wrench0.torque.y)
	t.z=(1-a)*(wrench.torque.z)+(a)*(wrench0.torque.z)
	wrench=Wrench(force=f,torque=t)
	
def callback(data):
	global wrench
	global wrench0
	global alpha	
	global delay
	#global interpenetration
	if data.states==[]:
		if delay==0:
			wrench=Wrench()
		#	interpenetration=[0.0]	
		if delay==1:
		     delay=0
	else: 
		delay=1
		wrench=data.states[0].total_wrench
		lowpass(alpha)
		#rospy.loginfo(wrench)
		wrench0=wrench
		#interpenetration=[data.states[0].depths[-1]] #takes the last element of the depth
	if not rospy.is_shutdown() and pub.get_num_connections() >= 0 :
		pub.publish(WrenchStamped(data.header,wrench))
		#pub2.publish(StampedDepths(data.header,interpenetration))

def listener():
    rospy.init_node('contact_listener', anonymous=True)
    rospy.Subscriber("/chaser/hand_bumper", ContactsState, callback)
    rospy.spin()
def publisher():  
	#Register a new topic
	global pub
	global pub2
	pub= rospy.Publisher("/chaser/hand_contact_wrench",WrenchStamped, queue_size=1000)
	#pub2= rospy.Publisher("/chaser/hand_contact_depths",StampedDepths,queue_size=1000)
	
if __name__ == '__main__':
	#initialization of variables
	delay=0 # to avoid gaps in measurements
	wrench=Wrench()
	wrench0=Wrench()
	alpha=0.9
	#interpenetration=[0.0]
	try:
		publisher()
	except rospy.ROSInterruptException: 
		pass
	listener()
