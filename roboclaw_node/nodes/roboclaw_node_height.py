#!/usr/bin/env python

#Node that is used to control the height of the robot's linear actuators
from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater
import roboclaw_driver.roboclaw_driver as roboclaw
import rospy
import tf
import signal
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"

def timeout_handler(signum, frame):
    raise Exception("serial write hanging")

class Node:
    def __init__(self):

        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}
        
        rospy.init_node("roboclaw_node_height")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        self.dev_name = rospy.get_param("~dev", "/dev/ttyACM2") #may need to change the usb port

        self.baud_rate = int(rospy.get_param("~baud", "38400")) #may need to change the baud rate. see roboclaw usermanual

        self.address = int(rospy.get_param("~address", "129")) #roboclaw is current setup to use address 128 which is setting 7


        #Error Handle
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")
        
        # TODO need someway to check if address is correct
        try:
            roboclaw.Open(self.dev_name, self.baud_rate)
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")
        
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.FunctionDiagnosticTask("Vitals", self.check_vitals))

        try:
            version = roboclaw.ReadVersion(self.address)
        except Exception as e:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(e)
            pass

        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.logdebug(repr(version[1]))

        roboclaw.SpeedM1M2(self.address, 0, 0)
        roboclaw.ResetEncoders(self.address)

        self.MAX_SPEED = float(rospy.get_param("~max_speed", "127"))
        self.MAX_DUTY = float(rospy.get_param("~max_duty", "32767"))

        self.last_set_speed_time = rospy.get_rostime()

        self.height_vel_sub = rospy.Subscriber("roboclaw/height_vel", Twist, self.cmd_vel_callback)
        self.height_pub = rospy.Publisher("roboclaw/height", JointState, queue_size=10)
        
        self.m1_duty = 0
        self.m2_duty = 0
        
        rospy.sleep(1)

        rospy.loginfo("dev %s", self.dev_name)
        rospy.loginfo("baud %d", self.baud_rate)
        rospy.loginfo("address %d", self.address)
        rospy.loginfo("max_speed %f", self.MAX_SPEED)

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(5)
        signal.signal(signal.SIGALRM, timeout_handler)
        
        while not rospy.is_shutdown():
            signal.setitimer(signal.ITIMER_REAL, 0.21, 0)
            
            try:
                if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 0.3:
                    self.m1_duty = 0
                    self.m2_duty = 0
                roboclaw.DutyM1M2(self.address,self.m1_duty,self.m2_duty)
                self.pub_enc_values()
                r_time.sleep()
                
            except Exception as e:
                rospy.loginfo(e)

    def pub_enc_values(self):
            height_state = JointState()
            height_state.header = Header()
            height_state.header.stamp = rospy.Time.now()
            height_state.name = ['m1', 'm2']
            enc1 = roboclaw.ReadEncM1(self.address)
            enc2 = roboclaw.ReadEncM2(self.address)
            height_state.position = [float(enc1[1]),float(enc2[1])]
            try:
                self.height_pub.publish(height_state)
            except Exception as e:
                rospy.loginfo("hanging")
                rospy.logdebug(e)

    def cmd_vel_callback(self, twist):
        #twist 127 full forward, -127 full backward
        self.last_set_speed_time = rospy.get_rostime() 
        self.m1_duty = self.twist_to_duty(twist.linear.x)
        self.m2_duty = self.twist_to_duty(twist.linear.y)
            
    def twist_to_duty(self, twist):
        return int((twist / self.MAX_SPEED) * self.MAX_DUTY)

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = roboclaw.ReadError(self.address)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10))
            stat.add("Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10))
            stat.add("Temp1 C:", float(roboclaw.ReadTemp(self.address)[1] / 10))
            stat.add("Temp2 C:", float(roboclaw.ReadTemp2(self.address)[1] / 10))
                       
            
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        self.height_vel_sub.unregister()
        try:
            roboclaw.ForwardM1(self.address, 0)
            roboclaw.ForwardM2(self.address, 0)

        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
                
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
