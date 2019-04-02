#!/usr/bin/env	python
"""
    BSD 3-Clause License

    cmdvel_to_wheelvelocity.py - converts cmd_vel to  left wheel and right wheel velocity in rad/s.

    Copyright (c) 2018, Veera Ragav
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
wheel_radius = rospy.get_param(rospy.get_namespace() + 'wheel_radius', 0.05)
robot_width = rospy.get_param(rospy.get_namespace() + 'robot_width', 0.52)

class WheelVelocity:

    def __init__(self):
        rospy.init_node('cmdvel_to_wheelvel', anonymous=True)
        rospy.Subscriber("cmd_vel", Twist, self.fcn)
        self.left_pub = rospy.Publisher('/left/motor_speed', Float64, queue_size=10)
        self.right_pub = rospy.Publisher('/right/motor_speed', Float64, queue_size=10)

    def fcn(self, twist):
        self.left_speed = -1 * ((2*twist.linear.x) - (twist.angular.z*robot_width)) / (2*wheel_radius) #rad/s
    	self.right_speed = ((2*twist.linear.x) + (twist.angular.z*robot_width)) / (2*wheel_radius)
        self.left_pub.publish(self.left_speed)
        self.right_pub.publish(self.right_speed)

if __name__ == '__main__':
    x = WheelVelocity()
    rospy.spin()
