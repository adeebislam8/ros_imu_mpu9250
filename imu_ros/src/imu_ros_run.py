#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu


import os
import sys
import time
import smbus
import numpy as np


import MPU9250
import kalman 

import tf.transformations 
class IMU_MPU9250:
	def __init__(self):
		
		self.address = 0x68
		self.bus = smbus.SMBus(1)
		self.imu = MPU9250.MPU9250(self.bus, self.address)
		self.imu_publisher = rospy.Publisher("imu_data", Imu, queue_size=10)
		self.imu_data = Imu()
			
		rospy.init_node("imu_node")

	def run(self):
		self.imu.begin()

		self.imu.loadCalibDataFromFile("/home/pi/catkin_ws/src/imu_ros/data/calib.json")

		self.sensorfusion = kalman.Kalman()

		self.imu.readSensor()
		self.imu.computeOrientation()
		self.sensorfusion.roll = self.imu.roll
		self.sensorfusion.pitch = self.imu.pitch
		self.sensorfusion.yaw = self.imu.yaw

		count = 0
		currTime = time.time()
		while True:
			self.imu.readSensor()
			# imu.computeOrientation()
			newTime = time.time()
			dt = newTime - currTime
			currTime = newTime

			self.sensorfusion.computeAndUpdateRollPitchYaw(self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2], self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2],\
														self.imu.MagVals[0], self.imu.MagVals[1], self.imu.MagVals[2], dt)

			# print ("Accel x: {0} ; Accel y : {1} ; Accel z : {2}".format(self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2]))
			# print ("Gyro x: {0} ; Gyro y : {1} ; Gyro z : {2}".format(self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2]))

			# print("Kalmanroll:{0} KalmanPitch:{1} KalmanYaw:{2} ".format(self.sensorfusion.roll, self.sensorfusion.pitch, self.sensorfusion.yaw))
			# print("KalmanrollCov:{0} KalmanPitchCov:{1} KalmanYawCov:{2} ".format(self.sensorfusion.rollCovariance, self.sensorfusion.pitchCovariance, self.sensorfusion.yawCovariance))
			quaternion = tf.transformations.quaternion_from_euler(self.sensorfusion.roll, self.sensorfusion.pitch, self.sensorfusion.yaw)
			self.imu_data.angular_velocity.x = self.imu.GyroVals[0]
			self.imu_data.angular_velocity.y = self.imu.GyroVals[1]
			self.imu_data.angular_velocity.z = self.imu.GyroVals[2]
			self.imu_data.linear_acceleration.x = self.imu.AccelVals[0]
			self.imu_data.linear_acceleration.y = self.imu.AccelVals[1]
			self.imu_data.linear_acceleration.z = self.imu.AccelVals[2]
			# print(quaternion)
			self.imu_data.orientation.x = quaternion[0]
			self.imu_data.orientation.y = quaternion[1]
			self.imu_data.orientation.z = quaternion[2]
			self.imu_data.orientation.w = quaternion[3]

			# print(self.imu_data)
			self.imu_publisher.publish(self.imu_data)
			time.sleep(0.01)
		
def main():
	imu_sensor = IMU_MPU9250()
	# while not rospy.is_shutdown():
	# 	imu_sensor.run()
	# 	rospy.spin()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		imu_sensor.run()
		imu_sensor.imu_publisher.publish(imu_sensor.imu_data)
		rate.sleep()

# Main loop
if __name__ == '__main__':
	main()	


