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
#!/usr/bin/env	python
import serial
import rospy
import time
import six
class ServoMotor:
    __port = ''
    __ser = serial.Serial(timeout=1)

    def __init__(self, serial_port_):
        ServoMotor.__port = serial_port_

    def openSerialPort(self):
        ServoMotor.__ser.port = ServoMotor.__port
        try:
            ServoMotor.__ser.open()
            rospy.loginfo('Opened the port %s', ServoMotor.__port)
        except:
            rospy.logfatal('Cannot open the port %s', ServoMotor.__port)
            rospy.signal_shutdown("Cannot open the port. Node shutting down.")

    def closeSerialPort(self):
        ServoMotor.__ser.close()

    def readMotorSpeed(self):
        ServoMotor.__ser.write('S\r')
        self.d = ServoMotor.__ser.readline()
        self.p = self.d.find('S')
        self.q = self.d.find(' ')
        self.d = self.d[self.p + 1 : self.q]
        self.d = int(self.d)
        return self.d

    def writeMotorSpeed(self, speed):
        rospy.logdebug('S'+str(int(speed)))
        ServoMotor.__ser.flushOutput()
        ServoMotor.__ser.write('S'+str(int(speed)))
        ServoMotor.__ser.flushInput()

    def getMaxMotorSpeed(self):
        self.p = 0
        self.q = 0
        self.firstPass = True
        while self.p is -1 or self.q is -1 or self.firstPass:
            self.firstPass = False
            ServoMotor.__ser.flushInput()
            ServoMotor.__ser.write('M\r')
            try:
                self.d = ServoMotor.__ser.readline()
            except:
                pass
            self.p = self.d.find('M')
            self.q = self.d.find(' ')
        self.d = self.d[self.p + 1 : self.q]
        self.d = int(self.d)
        ServoMotor.__ser.flushInput()
        return self.d

    def setMaxMotorSpeed(self, speed):
        try:
            ServoMotor.__ser.write('M'+str(int(speed)))
        except:
            pass

    def readDamping(self):
        try:
            ServoMotor.__ser.write('D\r')
            self.d = ServoMotor.__ser.readline()
            self.p = self.d.find('D')
            self.q = self.d.find(' ')
            self.d = self.d[self.p + 1 : self.q]
            self.d = int(self.d)
            return self.d
        except:
            pass

    def writeDamping(self, value):
        try:
            ServoMotor.__ser.write('D'+str(int(value)))
        except:
            pass

    def loadFactorySettings(self):
        try:
            ServoMotor.__ser.write('Y')
        except:
            pass

    def getPositionEncoder(self):
        self.p = 0
        self.q = 0
        self.d = ''
        self.firstPass = True
        while self.p is -1 or self.q is -1 or self.firstPass:
            self.firstPass = False
            ServoMotor.__ser.flushInput()
            ServoMotor.__ser.write('P\r')
            try:
                self.d = ServoMotor.__ser.readline()
                #rospy.loginfo(self.d)
                self.d = str(self.d)
                self.p = self.d.find('P')
                self.q = self.d.find(' ')
            except:
                return 0
        try:
            self.d = self.d[self.p + 1 : self.q]
            self.d = int(self.d)
            #rospy.loginfo(self.d)
            if not self.d or isinstance(self.d, six.string_types):
                return 0
            return self.d
        except:
            return 0

    def setPositionEncoder(self, encoder):
        try:
            ServoMotor.__ser.write('P'+str(encoder))
        except:
            pass

    def getAbsolutePostion(self):
        try:
            ServoMotor.__ser.write('G\r')
            self.d = ServoMotor.__ser.readline()
            self.p = self.d.find('G')
            self.q = self.d.find(' ')
            self.d = self.d[self.p + 1 : self.q]
            self.d = int(self.d)
            return self.d
        except:
            pass

    def setAbsolutePostion(self, position):
        try:
            rospy.logdebug('G'+str(int(position)))
            ServoMotor.__ser.flushOutput()
            ServoMotor.__ser.write('G'+str(int(position)))
            ServoMotor.__ser.flushInput()
        except:
            pass

    def setRelativePostion(self, position):
        try:
            rospy.logdebug('R'+str(int(position)))
            ServoMotor.__ser.flushOutput()
            ServoMotor.__ser.write('R'+str(int(position)))
            ServoMotor.__ser.flushInput()
        except:
            pass

    def getFeedbackGain(self):
        try:
            ServoMotor.__ser.write('A\r')
            self.d = ServoMotor.__ser.readline()
            self.p = self.d.find('A')
            self.q = self.d.find(' ')
            self.d = self.d[self.p + 1 : self.q]
            self.d = int(self.d)
            return self.d
        except:
            pass

    def setFeedbackGain(self, gain):
        try:
            ServoMotor.__ser.write('A'+str(gain))
        except:
            pass

    def getProportionateGain(self):
        try:
            ServoMotor.__ser.write('B\r')
            self.d = ServoMotor.__ser.readline()
            self.p = self.d.find('B')
            self.q = self.d.find(' ')
            self.d = self.d[self.p + 1 : self.q]
            self.d = int(self.d)
            return self.d
        except:
            pass

    def setProportionateGain(self, gain):
        try:
            ServoMotor.__ser.write('B'+str(gain))
        except:
            pass

    def getIntegralGain(self):
        try:
            ServoMotor.__ser.write('C\r')
            self.d = ServoMotor.__ser.readline()
            self.p = self.d.find('C')
            self.q = self.d.find(' ')
            self.d = self.d[self.p + 1 : self.q]
            self.d = int(self.d)
            return self.d
        except:
            pass

    def setIntegralGain(self, gain):
        try:
            ServoMotor.__ser.write('C'+str(gain))
        except:
            pass

    def autoCalibrate(self):
        try:
            ServoMotor.__ser.write('X')
        except:
            pass

    def isAvailable(self):
        ServoMotor.__ser.flushInput()
        ServoMotor.__ser.write('S\r')
        time.sleep(1)
        self.p = ServoMotor.__ser.inWaiting()
        if self.p == 0:
            rospy.logfatal("Motor Is Down at %s", ServoMotor.__port)
            #pass
        else:
            rospy.loginfo("Succesfully connected to motor in %s", ServoMotor.__port)
