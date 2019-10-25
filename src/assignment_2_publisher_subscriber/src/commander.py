import rospy
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import Speed

steering = 1.0
speed = 0.3

def callback(data):
	rospy.loginfo('Speed is %f', Speed(data.data).value)

	#rospy.loginfo('Speed is %f m/s', data.value)

def commander():
	norm_steering_left = NormalizedSteeringCommand()
	norm_steering_left.value = steering
	move_speed = Speed()
	move_speed.value = speed

	publisher_steering = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)

	rospy.init_node('commander', anonymous=True)

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		publisher_steering.publish(norm_steering_left)

if __name__ == '__main__':
     try:
         commander()
     except rospy.ROSInterruptException:
         pass
