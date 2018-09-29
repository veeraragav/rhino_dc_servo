#!/usr/bin/env	python
import rospy
from rhino_driver import ServoMotor
import serial
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from rhino_dc_servo.srv import *

class Rhino:
    __myMotor = ServoMotor(rospy.get_param(rospy.get_namespace() + 'port'))

    def __init__(self):
        rospy.init_node('motor', anonymous=True)
        self.encoder_pub = rospy.Publisher('encoderTicks', Int32, queue_size=10)
        Rhino.__myMotor.openSerialPort()
        Rhino.__myMotor.isAvailable()
        rospy.Subscriber('motor_speed', Float64, self.setSpeed)
        rospy.Subscriber('absolute_cmd', Float64, self.setAbsolute)
        rospy.Subscriber('relative_cmd', Float64, self.setRelative)
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
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def update(self):
        self.data = Int32()
        self.data.data = Rhino.__myMotor.getPositionEncoder()
        self.encoder_pub.publish(self.data)

    def __del__(self):
        Rhino.__myMotor.closeSerialPort()

    def setSpeed(self, msg):
        self.speed = msg.data * 9.5492965855137 * 65000 /rospy.get_param(rospy.get_namespace() + 'rpm') /Rhino.__myMotor.getMaxMotorSpeed()
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
