#! /usr/bin/env python

""" data_process_calibration_Es.py : This program aims to read data from output pose file and process the data for diff_drive_controller.
    Written by Cui
"""
import rospy
from math import sqrt, pow

class dataprocessEs():
	"""class for processing data from output pose files"""
	def __init__(self):

		self.nominal_wheel_diameter = 0.125
		self.nominal_wheel_seperation = 0.445208
		self.translation_length = 1

		#read data from laser tracker files
		#experiment 1:
		with open("/home/ros_match/1/start_pose_1.txt") as f: #/home/rosmatch/real_pose_clockwise.txt
			start_pose_1 = f.read()
			start_pose_1 = start_pose_1.split()
		self.xA1_1 = float(start_pose_1[2])
		self.yA1_1 = float(start_pose_1[3])
		self.xB1_1 = float(start_pose_1[0])
		self.yB1_1 = float(start_pose_1[1])
		
		with open("/home/ros_match/1/end_pose_1.txt") as f: #/home/rosmatch/real_pose_clockwise.txt
			end_pose_1 = f.read()
			end_pose_1 = end_pose_1.split()
		self.xA2_1 = float(end_pose_1[2])
		self.yA2_1 = float(end_pose_1[3])
		self.xB2_1 = float(end_pose_1[0])
		self.yB2_1 = float(end_pose_1[1])

		#experiment 2:
		with open("/home/ros_match/1/start_pose_2.txt") as f: #/home/rosmatch/real_pose_clockwise.txt
			start_pose_2 = f.read()
			start_pose_2 = start_pose_2.split()
		self.xA1_2 = float(start_pose_2[2])
		self.yA1_2 = float(start_pose_2[3])
		self.xB1_2 = float(start_pose_2[0])
		self.yB1_2 = float(start_pose_2[1])
		
		with open("/home/ros_match/1/end_pose_2.txt") as f: #/home/rosmatch/real_pose_clockwise.txt
			end_pose_2 = f.read()
			end_pose_2 = end_pose_2.split()
		self.xA2_2 = float(end_pose_2[2])
		self.yA2_2 = float(end_pose_2[3])
		self.xB2_2 = float(end_pose_2[0])
		self.yB2_2 = float(end_pose_2[1])

		#experiment 3:
		with open("/home/ros_match/1/start_pose_3.txt") as f: #/home/rosmatch/real_pose_clockwise.txt
			start_pose_3 = f.read()
			start_pose_3 = start_pose_3.split()
		self.xA1_3 = float(start_pose_3[2])
		self.yA1_3 = float(start_pose_3[3])
		self.xB1_3 = float(start_pose_3[0])
		self.yB1_3 = float(start_pose_3[1])
		
		with open("/home/ros_match/1/end_pose_3.txt") as f: #/home/rosmatch/real_pose_clockwise.txt
			end_pose_3 = f.read()
			end_pose_3 = end_pose_3.split()
		self.xA2_3 = float(end_pose_3[2])
		self.yA2_3 = float(end_pose_3[3])
		self.xB2_3 = float(end_pose_3[0])
		self.yB2_3 = float(end_pose_3[1])

		#experiment 4:
		with open("/home/ros_match/1/start_pose_4.txt") as f: #/home/rosmatch/real_pose_clockwise.txt
			start_pose_4 = f.read()
			start_pose_4 = start_pose_4.split()
		self.xA1_4 = float(start_pose_4[2])
		self.yA1_4 = float(start_pose_4[3])
		self.xB1_4 = float(start_pose_4[0])
		self.yB1_4 = float(start_pose_4[1])
		
		with open("/home/ros_match/1/end_pose_4.txt") as f: #/home/rosmatch/real_pose_clockwise.txt
			end_pose_4 = f.read()
			end_pose_4 = end_pose_4.split()
		self.xA2_4 = float(end_pose_4[2])
		self.yA2_4 = float(end_pose_4[3])
		self.xB2_4 = float(end_pose_4[0])
		self.yB2_4 = float(end_pose_4[1])

		#experiment 5:
		with open("/home/ros_match/1/start_pose_5.txt") as f: #/home/rosmatch/real_pose_clockwise.txt
			start_pose_5 = f.read()
			start_pose_5 = start_pose_5.split()
		self.xA1_5 = float(start_pose_5[2])
		self.yA1_5 = float(start_pose_5[3])
		self.xB1_5 = float(start_pose_5[0])
		self.yB1_5 = float(start_pose_5[1])
		
		with open("/home/ros_match/1/end_pose_5.txt") as f: #/home/rosmatch/real_pose_clockwise.txt
			end_pose_5 = f.read()
			end_pose_5 = end_pose_5.split()
		self.xA2_5 = float(end_pose_5[2])
		self.yA2_5 = float(end_pose_5[3])
		self.xB2_5 = float(end_pose_5[0])
		self.yB2_5 = float(end_pose_5[1])

		#compute the position_diff between start and end
		#experiment 1:
		self.position_diff_A_1 = sqrt(pow((self.yA2_1-self.yA1_1), 2)+pow((self.xA2_1-self.xA1_1), 2))
		self.position_diff_B_1 = sqrt(pow((self.yB2_1-self.yB1_1), 2)+pow((self.xB2_1-self.xB1_1), 2))
		self.position_diff_1 = (self.position_diff_A_1+self.position_diff_B_1)/2

		#experiment 2:
		self.position_diff_A_2 = sqrt(pow((self.yA2_2-self.yA1_2), 2)+pow((self.xA2_2-self.xA1_2), 2))
		self.position_diff_B_2 = sqrt(pow((self.yB2_2-self.yB1_2), 2)+pow((self.xB2_2-self.xB1_2), 2))
		self.position_diff_2 = (self.position_diff_A_2+self.position_diff_B_2)/2

		#experiment 3:
		self.position_diff_A_3 = sqrt(pow((self.yA2_3-self.yA1_3), 2)+pow((self.xA2_3-self.xA1_3), 2))
		self.position_diff_B_3 = sqrt(pow((self.yB2_3-self.yB1_3), 2)+pow((self.xB2_3-self.xB1_3), 2))
		self.position_diff_3 = (self.position_diff_A_3+self.position_diff_B_3)/2

		#experiment 4:
		self.position_diff_A_4 = sqrt(pow((self.yA2_4-self.yA1_4), 2)+pow((self.xA2_4-self.xA1_4), 2))
		self.position_diff_B_4 = sqrt(pow((self.yB2_4-self.yB1_4), 2)+pow((self.xB2_4-self.xB1_4), 2))
		self.position_diff_4 = (self.position_diff_A_4+self.position_diff_B_4)/2

		#experiment 5:
		self.position_diff_A_5 = sqrt(pow((self.yA2_5-self.yA1_5), 2)+pow((self.xA2_5-self.xA1_5), 2))
		self.position_diff_B_5 = sqrt(pow((self.yB2_5-self.yB1_5), 2)+pow((self.xB2_5-self.xB1_5), 2))
		self.position_diff_5 = (self.position_diff_A_5+self.position_diff_B_5)/2

		#average position_diff of 5 experiment in order to reduce the effect of non-systematic error
		self.position_diff = (self.position_diff_1+self.position_diff_2+self.position_diff_3+self.position_diff_4+self.position_diff_5)/5

		#compute Es and average wheel diameter
		self.Es = self.position_diff/self.translation_length
		self.average_wheel_diameter = self.Es*self.nominal_wheel_diameter
		self.average_wheel_radius = self.average_wheel_diameter/2

		#output Es in file
		self.list = []
		self.list.append(self.Es)
		self.list.append(self.average_wheel_diameter)
		self.list.append(self.average_wheel_radius)

		with open("Es_and_average_wheel_diameter.txt", 'w') as f:
			for j in self.list:
				f.write(str(j) + '\n')


if __name__ == '__main__':
	try:
		dataprocessEs()
	except Exception:
		print("Data processing failed")