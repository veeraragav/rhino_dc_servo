#!/usr/bin/env	python

"""
    BSD 3-Clause License

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
from rhino_driver import ServoMotor
import serial
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from rhino_dc_servo.srv import *

class Rhino:
    #instance of servomotor on a specified port
    __myMotor = ServoMotor(rospy.get_param(rospy.get_namespace() + 'port', '/dev/ttyUSB0'))
    __rpm = rospy.get_param(rospy.get_namespace() + 'rpm', 200)
    #constructor
    def __init__(self):
        rospy.init_node('motor', anonymous=True)

        #setting up encoder data publisher
        self.encoder_pub = rospy.Publisher('encoderTicks', Int32, queue_size=10)

        Rhino.__myMotor.openSerialPort()
        Rhino.__myMotor.isAvailable()

        #setting up subscribers
        rospy.Subscriber('motor_speed', Float64, self.setSpeed)
        rospy.Subscriber('absolute_cmd', Float64, self.setAbsolute)
        rospy.Subscriber('relative_cmd', Float64, self.setRelative)

        #setting up services
        self.read_damping = rospy.Service('read_damping', readDamping, self.dampingRead)
        self.write_damping = rospy.Service('write_damping', writeDamping, self.dampingWrite)
        self.load_factory_settings = rospy.Service('load_factory_settings', loadFactorySettings, self.loadFactory)
        self.set_position_encoder = rospy.Service('set_position_encoder', setPositionEncoder, self.setPositionEnc)
        self.get_absolute_position = rospy.Service('get_absolute_position', getAbsolutePosition, self.getAbsPos)
        self.set_feedback_gain = rospy.Service('set_feedback_gain', setFeedbackGain, self.setFG)
        self.get_feedback_gain = rospy.Service('get_feedback_gain', getFeedbackGain, self.getFG)
        self.set_proportionate_gain = rospy.Service('set_proportionate_gain', setProportionateGain, self.setPG)
        self.get_proportionate_gain = rospy.Service('get_proportionate_gain', getProportionateGain, self.getPG)
        self.get_integral_gain = rospy.Service('get_integral_gain', getIntegralGain, self.getIG)
        self.set_integral_gain = rospy.Service('set_integral_gain', setIntegralGain, self.setIG)
        self.set_max_motor_speed = rospy.Service('set_max_motor_speed', setMaxMotorSpeed, self.setMaxSpeed )
        get_max_motor_speed = rospy.Service('get_max_motor_speed', getMaxMotorSpeed, self.getMaxSpeed)
        self.auto_calibrate = rospy.Service('auto_calibrate', autoCalibrate, self.autoC)

    def spin(self):
        r = rospy.Rate(rospy.get_param(rospy.get_namespace() + 'rate', 5))
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def update(self):
        self.data = Int32()
        self.data.data = Rhino.__myMotor.getPositionEncoder()
        if self.data.data:
            self.encoder_pub.publish(self.data)

    def __del__(self):
        Rhino.__myMotor.closeSerialPort()

    def setSpeed(self, msg):
        rospy.loginfo(msg)
        self.speed = msg.data * 9.5492965855137 * 65000 /Rhino.__rpm /Rhino.__myMotor.getMaxMotorSpeed()
        Rhino.__myMotor.writeMotorSpeed(self.speed)

    def setAbsolute(self, msg):
        self.position = msg.data * 286.4788976
        Rhino.__myMotor.setAbsolutePostion(self.position)

    def setRelative(self, msg):
        self.position = msg.data * 286.4788976
        Rhino.__myMotor.setRelativePostion(self.position)

    def dampingRead(self, request):
        return readDampingResponse(Rhino.__myMotor.readDamping())

    def dampingWrite(self, request):
        Rhino.__myMotor.writeDamping(request.damp)
        return []

    def loadFactory(self, request):
        Rhino.__myMotor.loadFactorySettings()
        return []

    def setPositionEnc(self, request):
        Rhino.__myMotor.setPositionEncoder(request.encoder)
        return []

    def getAbsPos(self, request):
        return getAbsolutePositionResponse(Rhino.__myMotor.getAbsolutePostion())

    def setFG(self, request):
        Rhino.__myMotor.setFeedbackGain(request.gain)
        return []

    def getFG(self, request):
        return getFeedbackGainResponse(Rhino.__myMotor.getFeedbackGain())

    def setPG(self, request):
        Rhino.__myMotor.setProportionateGain(request.pgain)
        return []

    def getPG(self, request):
        return getProportionateGainResponse(Rhino.__myMotor.getProportionateGain())

    def setIG(self, request):
        Rhino.__myMotor.setIntegralGain(request.igain)
        return []

    def getIG(self, request):
        return getIntegralGainResponse(Rhino.__myMotor.getIntegralGain())

    def setMaxSpeed(self, request):
        Rhino.__myMotor.setMaxMotorSpeed(request.mspeed)
        return []

    def getMaxSpeed(self, request):
        return getMaxMotorSpeedResponse(Rhino.__myMotor.getMaxMotorSpeed())

    def autoC(self, request):
        Rhino.__myMotor.autoCalibrate()
        return []
if __name__ == '__main__':
    a = Rhino()
    a.spin()
