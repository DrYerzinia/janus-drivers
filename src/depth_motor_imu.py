#!/usr/bin/python3

import sys
import time
import threading

import rospy

from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

import board
import busio
import adafruit_bno055
from adafruit_servokit import ServoKit
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

def initalizePressureSensor(i2c):

    result = bytearray(2)

    C1 = 0
    C2 = 0
    C3 = 0
    C4 = 0
    C5 = 0
    C6 = 0

    try:

        while not i2c.try_lock():
            # TODO timeout
            pass

        # Reset pressure sensor
        i2c.writeto(0x76, bytes([0x1E]), stop=True)

        # Wait for pressure sensor to be ready
        time.sleep(0.1)

        # Read PROM calibration data
        i2c.writeto(0x76, bytes([0xA2]), stop=False)
        i2c.readfrom_into(0x76, result)
        C1 = int.from_bytes(result, 'big', signed=False)

        i2c.writeto(0x76, bytes([0xA4]), stop=False)
        i2c.readfrom_into(0x76, result)
        C2 = int.from_bytes(result, 'big', signed=False)

        i2c.writeto(0x76, bytes([0xA6]), stop=False)
        i2c.readfrom_into(0x76, result)
        C3 = int.from_bytes(result, 'big', signed=False)

        i2c.writeto(0x76, bytes([0xA8]), stop=False)
        i2c.readfrom_into(0x76, result)
        C4 = int.from_bytes(result, 'big', signed=False)

        i2c.writeto(0x76, bytes([0xAA]), stop=False)
        i2c.readfrom_into(0x76, result)
        C5 = int.from_bytes(result, 'big', signed=False)

        i2c.writeto(0x76, bytes([0xAC]), stop=False)
        i2c.readfrom_into(0x76, result)
        C6 = int.from_bytes(result, 'big', signed=False)

        i2c.unlock()

    except OSError:
        # TODO print error
        pass
    except RuntimeError:
        # TODO print error
        pass

    return [0, C1, C2, C3, C4, C5, C6]

def readAndCalibratePressure(i2c, C):

    result = bytearray(3)

    TEMP = 0
    P = 0

    try:

        while not i2c.try_lock():
            # TODO timeout
            pass

        # Start Pressure Conversion
        i2c.writeto(0x76, bytes([0x48]), stop=True)

        time.sleep(0.01)

        # Read Pressure Result
        i2c.writeto(0x76, bytes([0x00]), stop=False)
        i2c.readfrom_into(0x76, result)
        D1 = int.from_bytes(result, 'big', signed=False)

        # Start Temperature Conversion
        i2c.writeto(0x76, bytes([0x58]), stop=True)

        time.sleep(0.01)

        # Read Pressure Result
        i2c.writeto(0x76, bytes([0x00]), stop=False)
        i2c.readfrom_into(0x76, result)
        D2 = int.from_bytes(result, 'big', signed=False)

        dT = D2 - C[5]*256.0
        TEMP = 2000.0 + dT*C[6]/8388608.0

        OFF = C[2]*65536.0 + (C[4]*dT)/128.0
        SENS = C[1]*32768.0 + (C[3]*dT)/256.0
        P = (D1*SENS/2097152.0 - OFF)/8192.0

        i2c.unlock()

    except OSError:
        pass
    except RuntimeError:
        pass

    return TEMP, P

# TODO load motor map from config
motor_map = \
  {
    "down_front_left":  4,
    "down_front_right": 3,
    "down_back_right":  0,
    "down_back_left":   6,
    "front_left":       5,
    "front_right":      2,
    "back_right":       7,
    "back_left":        1
  }

command_map = \
  {
    0: "down_front_left",
    1: "down_front_right",
    2: "down_back_right",
    3: "down_back_left",
    4: "front_left",
    5: "front_right",
    6: "back_right",
    7: "back_left"
  }

class DepthMotorIMU():

    def __init__(self):

        # Load parameters
        subname = "janus" #TODO unhardcode

        # Initalise I2C
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # Initalise Depth Sensor
        self.C = initalizePressureSensor(self.i2c)

        self.pub_odom = rospy.Publisher(
            'depth_odom', PoseWithCovarianceStamped, queue_size=1)
        self.pub_map = rospy.Publisher(
            'depth_map', PoseWithCovarianceStamped, queue_size=1)

        self.pub_odom_data = PoseWithCovarianceStamped()
        self.pub_odom_data.pose.covariance = [0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0.01, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0]
        self.pub_odom_data.header.frame_id = subname + "/description/depth_odom_frame"

        self.pub_map_data = PoseWithCovarianceStamped()
        self.pub_map_data.pose.covariance = [0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0.01, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0]
        self.pub_map_data.header.frame_id = subname + "/description/depth_map_frame"

        # Initalise IMU
        self.imu = adafruit_bno055.BNO055(self.i2c)

        # Setup orientation
        self.imu.mode = adafruit_bno055.CONFIG_MODE
        #         AXIS_REMAP_CONFIG    AXIS_REMAP_SIGN
        # ADDRESS 0x41                 0x42
        # VALUE   0x21                 0x01
        self.imu._write_register(0x41, 0x24)
        self.imu._write_register(0x42, 0x03)
        self.imu.mode = adafruit_bno055.NDOF_MODE

        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)

        self.imu_data = Imu()
        self.imu_data.header.frame_id = subname + "/description/imu_link"
        self.imu_data.orientation_covariance =         [1e-6,    0,    0,
                                                           0, 1e-6,    0,
                                                           0,    0, 1e-6]
        self.imu_data.angular_velocity_covariance =    [1e-6,    0,    0,
                                                           0, 1e-6,    0,
                                                           0,    0, 1e-6]
        self.imu_data.linear_acceleration_covariance = [1e-6,    0,    0,
                                                           0, 1e-6,    0,
                                                           0,    0, 1e-6]

        # Initalise PWM Controller
        self.motor = ServoKit(channels=8)
        rospy.loginfo("Initalizing ECSs...")
        for motor_name in motor_map.keys():

            motor_id = motor_map[motor_name]

            rospy.loginfo(motor_name)
            self.motor.servo[motor_id].set_pulse_width_range(1085, 2000)
            time.sleep(0.2)

        self.motor_command_sub = rospy.Subscriber('motor_controllers/pololu_control/command', Float64MultiArray, self.motor_command_callback, queue_size=1)

        # Initalize Battery Monitor
        # Create the ADC object using the I2C bus
        ads = ADS.ADS1115(self.i2c)

        # Create single-ended input on channel 0
        self.battery_monitor_channel = AnalogIn(ads, ADS.P0)

        self.bat_pub = rospy.Publisher(
            'battery_voltage', Float64, queue_size=1)
        self.bat_data = Float64()

        # Start Battery Voltage Publisher
        battery_thread = threading.Thread(target = self.battery_publisher)
        battery_thread.start()

        # Start Depth Publisher
        depth_thread = threading.Thread(target = self.depth_publisher)
        depth_thread.start()

        # Start IMU Publisher
        imu_thread = threading.Thread(target = self.imu_publisher)
        imu_thread.start()

        rospy.loginfo("Initalization Complete")

        # Run until we kill the core
        rospy.spin()

    def motor_command_callback(self, msg):

        for i in range(8):

            # Get throttle value from 0.0 to 1.0
            throttle = (msg.data[i] - 1000.0) / 1000.0
            if throttle <= 0.0:
                throttle = 0.0
            elif throttle >= 1.0:
                throttle = 1.0

            # TODO thrust limiter?

            motor_name = command_map[i]
            motor_id = motor_map[motor_name]
            self.motor.servo[motor_id].fraction = throttle

    def battery_publisher(self):

        r = rospy.Rate(0.5)

        while not rospy.is_shutdown():

            voltage = self.battery_monitor_channel.voltage

            # compensate for voltage divider
            battery_voltage = voltage * 7.8697 # voltage / ( 2.2 / (2.2 + 15.0))

            self.bat_data.data = battery_voltage

            self.bat_pub.publish(self.bat_data)

            r.sleep()

    def imu_publisher(self):

        r = rospy.Rate(10) # IMU 10Hz TODO pick good value

        while not rospy.is_shutdown():

            # Publish IMU Data
            quat = self.imu.quaternion
            self.imu_data.header.stamp = rospy.Time.now()
            self.imu_data.orientation = Quaternion()

            # BNO055 Produces quaternion in W, X, Y, Z order
            self.imu_data.orientation.x = quat[1]
            self.imu_data.orientation.y = quat[2]
            self.imu_data.orientation.z = quat[3]
            self.imu_data.orientation.w = quat[0]

            self.imu_pub.publish(self.imu_data)

            r.sleep()

    def depth_publisher(self):

        r = rospy.Rate(10) # Depth 10Hz TODO pick good value

        while not rospy.is_shutdown():

            # TODO zero offset calibration
            offset = 0.0

            # Publish Depth Data
            _, pressure = readAndCalibratePressure(self.i2c, self.C)
            pressure_mbar = pressure / 10.0
            depth_m = -1*(pressure_mbar-797.11)*100/(1030*9.8) + offset

            # rospy.loginfo("DEPTH %.2f %2.f" % (pressure_mbar, depth_m))

            # publish
            self.pub_odom_data.header.stamp = rospy.Time.now()
            self.pub_odom_data.header.seq += 1
            self.pub_odom_data.pose.pose.position.z = depth_m

            self.pub_map_data.header.stamp = rospy.Time.now()
            self.pub_map_data.header.seq += 1
            self.pub_map_data.pose.pose.position.z = depth_m

            self.pub_odom.publish(self.pub_odom_data)
            self.pub_map.publish(self.pub_map_data)

            r.sleep()


if __name__ == '__main__':

    rospy.init_node('depth_motor_imu')

    try:
        node = DepthMotorIMU()
    except rospy.ROSInterruptException:
        pass
