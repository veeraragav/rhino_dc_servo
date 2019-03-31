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
import swri_rospy
from rhino_driver import ServoMotor
import serial
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from rhino_dc_servo.srv import *

class Rhino:
    #constructor
    def __init__(self):
        rospy.init_node('motor', anonymous=True)

        #get params
        self.myMotor = ServoMotor(rospy.get_param(rospy.get_namespace() + 'port', '/dev/ttyUSB0'))
        self.rpm = rospy.get_param(rospy.get_namespace() + 'rpm', 200)
        self.r = rospy.get_param(rospy.get_namespace() + 'rate', 5)

        #setting up encoder data publisher
        self.encoder_pub = rospy.Publisher('encoderTicks', Int32, queue_size=10)
        swri_rospy.Timer(rospy.Duration(1.0/self.r), self.update) #TODO: Use rate parameter

        self.myMotor.openSerialPort()
        self.myMotor.isAvailable()

        self.maxSpeed = self.myMotor.getMaxMotorSpeed()

        #setting up subscribers
        swri_rospy.Subscriber('motor_speed', Float64, self.setSpeed)
        swri_rospy.Subscriber('absolute_cmd', Float64, self.setAbsolute)
        swri_rospy.Subscriber('relative_cmd', Float64, self.setRelative)

        #setting up services
        self.read_damping = swri_rospy.Service('read_damping', readDamping, self.dampingRead)
        self.write_damping = swri_rospy.Service('write_damping', writeDamping, self.dampingWrite)
        self.load_factory_settings = swri_rospy.Service('load_factory_settings', loadFactorySettings, self.loadFactory)
        self.set_position_encoder = swri_rospy.Service('set_position_encoder', setPositionEncoder, self.setPositionEnc)
        self.get_absolute_position = swri_rospy.Service('get_absolute_position', getAbsolutePosition, self.getAbsPos)
        self.set_feedback_gain = swri_rospy.Service('set_feedback_gain', setFeedbackGain, self.setFG)
        self.get_feedback_gain = swri_rospy.Service('get_feedback_gain', getFeedbackGain, self.getFG)
        self.set_proportionate_gain = swri_rospy.Service('set_proportionate_gain', setProportionateGain, self.setPG)
        self.get_proportionate_gain = swri_rospy.Service('get_proportionate_gain', getProportionateGain, self.getPG)
        self.get_integral_gain = swri_rospy.Service('get_integral_gain', getIntegralGain, self.getIG)
        self.set_integral_gain = swri_rospy.Service('set_integral_gain', setIntegralGain, self.setIG)
        self.set_max_motor_speed = swri_rospy.Service('set_max_motor_speed', setMaxMotorSpeed, self.setMaxSpeed )
        self.get_max_motor_speed = swri_rospy.Service('get_max_motor_speed', getMaxMotorSpeed, self.getMaxSpeed)
        self.auto_calibrate = swri_rospy.Service('auto_calibrate', autoCalibrate, self.autoC)

        #self.setup()

    def update(self, event):
        data = Int32()
        data.data = self.myMotor.getPositionEncoder()
        if data.data != None :
            self.encoder_pub.publish(data)

    def __del__(self):
        self.myMotor.closeSerialPort()

    def setSpeed(self, msg):
        #rospy.loginfo(msg)
        speed = msg.data * 9.5492965855137 * 65000 /self.rpm /self.maxSpeed #255
        self.myMotor.writeMotorSpeed(speed)

    def setAbsolute(self, msg):
        position = msg.data * 286.4788976
        self.myMotor.setAbsolutePostion(position)

    def setRelative(self, msg):
        position = msg.data * 286.4788976
        self.myMotor.setRelativePostion(position)

    def dampingRead(self, request):
        return readDampingResponse(self.myMotor.readDamping())

    def dampingWrite(self, request):
        self.myMotor.writeDamping(request.damp)
        return []

    def loadFactory(self, request):
        self.myMotor.loadFactorySettings()
        return []

    def setPositionEnc(self, request):
        self.myMotor.setPositionEncoder(request.encoder)
        return []

    def getAbsPos(self, request):
        return getAbsolutePositionResponse(self.myMotor.getAbsolutePostion())

    def setFG(self, request):
        self.myMotor.setFeedbackGain(request.gain)
        return []

    def getFG(self, request):
        return getFeedbackGainResponse(self.myMotor.getFeedbackGain())

    def setPG(self, request):
        self.myMotor.setProportionateGain(request.pgain)
        return []

    def getPG(self, request):
        return getProportionateGainResponse(self.myMotor.getProportionateGain())

    def setIG(self, request):
        self.myMotor.setIntegralGain(request.igain)
        return []

    def getIG(self, request):
        return getIntegralGainResponse(self.myMotor.getIntegralGain())

    def setMaxSpeed(self, request):
        self.myMotor.setMaxMotorSpeed(request.mspeed)
        return []

    def getMaxSpeed(self, request):
        return getMaxMotorSpeedResponse(self.myMotor.getMaxMotorSpeed())

    def autoC(self, request):
        self.myMotor.autoCalibrate()
        return []

    def setup(self):
        #set damping = 0, max_speed=255
        self.myMotor.writeDamping(0)
        self.myMotor.setMaxMotorSpeed(255)


if __name__ == '__main__':
    a = Rhino()
   # a.setup()
    swri_rospy.spin()
