import rospy

from autominy_msgs.msg import Speed

def callback(data):
	rospy.loginfo('Speed is %f', Speed(data.data).value)

	#rospy.loginfo('Speed is %f m/s', data.value)

def speedometer():
	rospy.init_node('speedometer', anonymous=True)

	rospy.Subscriber('/senors/speed', Speed, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
     try:
         speedometer()
     except rospy.ROSInterruptException:
         pass
