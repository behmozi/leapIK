#!/usr/bin/env python
import rospy
import leap_interface
import numpy as np
from leap_motion.msg import leap
from leap_motion.msg import leapros
from sensor_msgs.msg import JointState
def callback(msg):
	x=msg.index_tip.x/250
	y=msg.index_tip.y/250
	z=msg.index_tip.z/250
	js = JointState()
	js.name=['hip', 'shoulder', 'elbow']

	# write IK code here
	l1=0.5
	l2=0.4
	l3=0.4
	theta1 = np.arctan2(x,z)
	theta3 = np. arccos(((y-l1)**2+(x/np.cos(theta1))**2-l2**2-l3**2)/ (2*l2*l3))
	betha = np.arctan2(l2+l3*np.cos(theta3),l3*np.sin(theta3))
	theta2 = np.arctan2(x/np.cos(theta1),(y-l1))-betha
        #theta1 = np.arctan2(-x,y)
	#c3 = ((z-l1)**2+  (y/np.cos(theta1))**2-l2**2-l3**2)/ (2*l2*l3)
        #s3 = np.sqrt(np.abs(1-c3**2))
              
        #theta3 = np.arctan2(s3,c3)

        #k1 = l2+l3*np.cos(theta3)
        #k2 = l3*np.sin(theta3)

        #P = 1+np.cos(theta1)/y*k2
        #Q = -2*k1*np.cos(theta1)/y
        #R = 1-k2*np.cos(theta1)/y

        #theta2 = np.arctan2(-Q+np.sqrt(np.abs(Q**2-4*P*R)),2*P)

	hip=theta1
	shoulder=theta2
	elbow=theta3
	#-----------------------------------------------------------------------
	js.position=[hip,shoulder,elbow]
	#js.position=[1,1,1]
	#print 'Real:', msg.index_tip.x
	#print
	jsp.publish(js)
rospy.init_node('msub')
sub = rospy.Subscriber('/leapmotion/data', leapros, callback)
jsp = rospy.Publisher('/main/joint_states', JointState, queue_size=1)

rospy.spin()

