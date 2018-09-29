#!/usr/bin/env	python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
radius = rospy.get_param(rospy.get_namespace() + 'wheel_radius')
width = rospy.get_param(rospy.get_namespace() + 'robot_width')

class WheelVelocity:

    def __init__(self):
        rospy.init_node('cmdvel_to_wheelvel', anonymous=True)
        rospy.Subscriber("cmd_vel", Twist, self.fcn)
        self.left_pub = rospy.Publisher('/left/motor_speed', Float64, queue_size=10)
        self.right_pub = rospy.Publisher('/right/motor_speed', Float64, queue_size=10)
    def fcn(self, msg):
        self.left_speed = -1 * (msg.linear.x - msg.angular.z*width/2) / radius
    	self.right_speed = (msg.linear.x + msg.angular.z*width/2) / radius
        self.left_pub.publish(self.left_speed)
        self.right_pub.publish(self.right_speed)

if __name__ == '__main__':
    x = WheelVelocity()
    rospy.spin()
