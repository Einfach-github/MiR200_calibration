#! /usr/bin/env python

""" data_process_calibration_Eb_Ed.py : This program aims to read data from output pose file and process the data for diff_drive_controller.
    Written by Cui
"""
import rospy
from math import sin, cos, pi, atan, sqrt, pow

class dataprocessEbEd():
	"""class for processing data from output pose files"""
	def __init__(self):

		self.nominal_wheel_diameter = 0.0625*2
		self.nominal_wheel_seperation = 0.445208
		self.sqaure_length = 4

		#read data from laser tracker files
		#experiment 1:
		with open("give the file location like...") as f: #/home/rosmatch/real_pose_clockwise.txt
			start_pose_cw_1 = f.read()
			start_pose_cw_1 = start_pose_cw_1.split()
		self.xA1_cw_1 = float(start_pose_cw_1[xA])
		self.yA1_cw_1 = float(start_pose_cw_1[yA])
		self.xB1_cw_1 = float(start_pose_cw_1[xB])
		self.yB1_cw_1 = float(start_pose_cw_1[yB])
		
		with open("give the file location like...") as f: #/home/rosmatch/real_pose_clockwise.txt
			end_pose_cw_1 = f.read()
			end_pose_cw_1 = end_pose_cw_1.split()
		self.xA2_cw_1 = float(end_pose_cw_1[xA])
		self.yA2_cw_1 = float(end_pose_cw_1[yA])
		self.xB2_cw_1 = float(end_pose_cw_1[xB])
		self.yB2_cw_1 = float(end_pose_cw_1[yB])

		with open("give the file location like...") as f: #/home/rosmatch/real_pose_counterclockwise.txt
			start_pose_ccw_1 = f.read()
			start_pose_ccw_1 = start_pose_ccw_1.split()
		self.xA1_ccw_1 = float(start_pose_ccw_1[xA])
		self.yA1_ccw_1 = float(start_pose_ccw_1[yA])
		self.xB1_ccw_1 = float(start_pose_ccw_1[xB])
		self.yB1_ccw_1 = float(start_pose_ccw_1[yB])

		with open("give the file location like...") as f: #/home/rosmatch/real_pose_counterclockwise.txt
			end_pose_ccw_1 = f.read()
			end_pose_ccw_1 = end_pose_ccw_1.split()
		self.xA2_ccw_1 = float(end_pose_ccw_1[xA])
		self.yA2_ccw_1 = float(end_pose_ccw_1[yA])
		self.xB2_ccw_1 = float(end_pose_ccw_1[xB])
		self.yB2_ccw_1 = float(end_pose_ccw_1[yB])

		#experiment 2:
		#experiment 3:
		#experiment 4:
		#experiment 5:

		#compute the change of positon x, y and the change of orientation between start and end in both counterclockwise and clockwise directions(in experiment)
		#experiment 1:
		#clockwise:
		self.x_diff_cw_1 = (self.xA2_cw_1+self.xB2_cw_1)/2 - (self.xA1_cw_1+self.xB1_cw_1)/2
		self.y_diff_cw_1 = (self.yA2_cw_1+self.yB2_cw_1)/2 - (self.yA1_cw_1+self.yB1_cw_1)/2
		self.position_diff_cw_1 = sqrt(pow(self.x_diff_cw_1, 2) + pow(self.y_diff_cw_1, 2))

		self.xA1_t_cw_1 = self.xA1_cw_1-self.xB1_cw_1
		self.yA1_t_cw_1 = self.yA1_cw_1-self.yB1_cw_1
		self.theta_cw_1 = atan(abs(self.yA1_t_cw_1/self.xA1_t_cw_1))
		if (self.xA1_t_cw_1 > 0):
			if (self.yA1_t_cw_1 > 0):
				self.alpha_cw_1 = self.theta_cw_1
			if (self.yA1_t_cw_1 < 0):
				self.alpha_cw_1 = -self.theta_cw_1

		if (self.xA1_t_cw_1 < 0):
			if (self.yA1_t_cw_1 < 0):
				self.alpha_cw_1 = self.theta_cw_1 - pi
			if (self.yA1_t_cw_1 > 0):
				self.alpha_cw_1 = -self.theta_cw_1 + pi
		
		self.xA2_t_cw_1 = self.xA2_cw_1 - self.xB2_cw_1
		self.yA2_t_cw_1 = self.yA2_cw_1 - self.yB2_cw_1
		self.xA2_r_cw_1 = cos(-self.alpha_cw_1)*self.xA2_t_cw_1 - sin(-self.alpha_cw_1)*self.yA2_t_cw_1
		self.yA2_r_cw_1 = sin(-self.alpha_cw_1)*self.xA2_t_cw_1 + cos(-self.alpha_cw_1)*self.yA2_t_cw_1
		self.beta_cw_1 = atan(abs(self.yA2_r_cw_1/self.xA2_r_cw_1))
		if (self.xA2_r_cw_1 > 0 ):
			if (self.yA2_r_cw_1 > 0):
				self.orientation_diff_cw_1 = self.beta_cw_1
			if (self.yA2_r_cw_1 < 0):
				self.orientation_diff_cw_1 = -self.beta_cw_1

		if (self.xA2_r_cw_1 < 0 ):
			if (self.yA2_r_cw_1 < 0):
				self.orientation_diff_cw_1 = self.beta_cw_1 - pi
			if (self.yA2_r_cw_1 > 0):
				self.orientation_diff_cw_1 = -self.beta_cw_1 + pi

		#counterclockwise:
		self.x_diff_ccw_1 = (self.xA2_ccw_1+self.xB2_ccw_1)/2 - (self.xA1_ccw_1+self.xB1_ccw_1)/2
		self.y_diff_ccw_1 = (self.yA2_ccw_1+self.yB2_ccw_1)/2 - (self.yA1_ccw_1+self.yB1_ccw_1)/2
		self.position_diff_ccw_1 = sqrt(pow(self.x_diff_ccw_1, 2) + pow(self.y_diff_ccw_1, 2))

		self.xA1_t_ccw_1 = self.xA1_ccw_1-self.xB1_ccw_1
		self.yA1_t_ccw_1 = self.yA1_ccw_1-self.yB1_ccw_1
		self.theta_ccw_1 = atan(abs(self.yA1_t_ccw_1/self.xA1_t_ccw_1))
		if (self.xA1_t_ccw_1 > 0):
			if (self.yA1_t_ccw_1 > 0):
				self.alpha_ccw_1 = self.theta_ccw_1
			if (self.yA1_t_ccw_1 < 0):
				self.alpha_ccw_1 = -self.theta_ccw_1

		if (self.xA1_t_ccw_1 < 0):
			if (self.yA1_t_ccw_1 < 0):
				self.alpha_ccw_1 = self.theta_ccw_1 - pi
			if (self.yA1_t_ccw_1 > 0):
				self.alpha_ccw_1 = -self.theta_ccw_1 + pi
		
		self.xA2_t_ccw_1 = self.xA2_ccw_1 - self.xB2_ccw_1
		self.yA2_t_ccw_1 = self.yA2_ccw_1 - self.yB2_ccw_1
		self.xA2_r_ccw_1 = cos(-self.alpha_ccw_1)*self.xA2_t_ccw_1 - sin(-self.alpha_ccw_1)*self.yA2_t_ccw_1
		self.yA2_r_ccw_1 = sin(-self.alpha_ccw_1)*self.xA2_t_ccw_1 + cos(-self.alpha_ccw_1)*self.yA2_t_ccw_1
		self.beta_ccw_1 = atan(abs(self.yA2_r_ccw_1/self.xA2_r_ccw_1))
		if (self.xA2_r_ccw_1 > 0 ):
			if (self.yA2_r_ccw_1 > 0):
				self.orientation_diff_ccw_1 = self.beta_ccw_1
			if (self.yA2_r_ccw_1 < 0):
				self.orientation_diff_ccw_1 = -self.beta_ccw_1

		if (self.xA2_r_ccw_1 < 0 ):
			if (self.yA2_r_ccw_1 < 0):
				self.orientation_diff_ccw_1 = self.beta_ccw_1 - pi
			if (self.yA2_r_ccw_1 > 0):
				self.orientation_diff_ccw_1 = -self.beta_ccw_1 + pi

		#experiment 2:
		#experiment 3:
		#experiment 4:
		#experiment 5:

		#average x_diff, y_diff and orientation_diff of 5 experiment in order to reduce the effect of non-systematic error (in cw and ccw)
		self.x_diff_clockwise = (self.x_diff_cw_1+self.x_diff_cw_2+self.x_diff_cw_3+self.x_diff_cw_4+self.x_diff_cw_5)/5
		self.y_diff_clockwise = (self.y_diff_cw_1+self.y_diff_cw_2+self.y_diff_cw_3+self.y_diff_cw_4+self.y_diff_cw_5)/5
		self.orientation_diff_clockwise = (self.orientation_diff_cw_1+self.orientation_diff_cw_2+self.orientation_diff_cw_3+self.orientation_diff_cw_4+self.orientation_diff_cw_5)/5

		self.x_diff_counterclockwise = (self.x_diff_ccw_1+self.x_diff_ccw_2+self.x_diff_ccw_3+self.x_diff_ccw_4+self.x_diff_ccw_5)/5
		self.y_diff_counterclockwise = (self.y_diff_ccw_1+self.y_diff_ccw_2+self.y_diff_ccw_3+self.y_diff_ccw_4+self.y_diff_ccw_5)/5
		self.orientation_diff_counterclockwise = (self.orientation_diff_ccw_1+self.orientation_diff_ccw_2+self.orientation_diff_ccw_3+self.orientation_diff_ccw_4+self.orientation_diff_ccw_5)/5

		#output x_diff, y_diff, position_diff and orientation_diff in file (in cw and ccw)
		self.pose_cw = []
		self.pose_ccw = []

		#experiment 1:
		#clockwise:
		self.pose_cw.append(self.x_diff_cw_1)
		self.pose_cw.append(self.y_diff_cw_1)
		self.pose_cw.append(self.position_diff_cw_1)
		self.pose_cw.append(self.orientation_diff_cw_1)
		#counterclockwise:
		self.pose_ccw.append(self.x_diff_ccw_1)
		self.pose_ccw.append(self.y_diff_ccw_1)
		self.pose_ccw.append(self.position_diff_ccw_1)
		self.pose_ccw.append(self.orientation_diff_ccw_1)

		#experiment 2:
		#experiment 3:
		#experiment 4:
		#experiment 5:

		with open("pose_info_cw.txt", 'w') as f:
			for j in self.pose_cw:
				f.write(str(j) + '\n')

		with open("pose_info_ccw.txt", 'w') as f:
			for j in self.pose_ccw:
				f.write(str(j) + '\n')

		"""If we use mir_200_v1.urdf.xacro, the model contains no errors. For simulation we give initially and manuelly the wheel diameter error 
		and wheelbase error for diff_drive_controller to let the robot running with errors by giving simulation compensation factors.

		However, if we use mir_200_v1_calibration.urdf.xacro, the model contains already the errors, then we do not need to give the compensation 
		factors for diff_drive_controller to let the robot running with errors."""
		self.sim_Ed = 1.00
		self.sim_Eb = 1.00

		self.state_1 = self.SimulationCompensationFactors(self.sim_Ed, self.sim_Eb)
		if self.state_1 == True:
			print("Simulation compensation factors have been calculated and saved in file simulation_compensation_factors.txt")
		else:
			print("Simulation compensation factors have failed to calculate.")

		self.state_2 = self.NewMethodCalibration(self.orientation_diff_clockwise, self.orientation_diff_counterclockwise, self.nominal_wheel_seperation, self.sqaure_length)
		if self.state_2 == True:
			print("New method calibration factors have been calculated and saved in file new_method_calibration_factors.txt")
		else:
			print("New method calibration factors have failed to calculate.")	
	
		self.state_3 = self.UMBmarkCalibration(self.x_diff_clockwise, self.x_diff_counterclockwise, self.y_diff_clockwise, self.y_diff_counterclockwise, self.nominal_wheel_seperation, self.sqaure_length)
		if self.state_3 == True:
			print("UMBmark method calibration factors have been calculated and saved in file UMBmark_method_calibration_factors.txt")
		else:
			print("UMBmark method calibration factors have failed to calculate.")	
	
		self.state_4 = self.LeeCalibration(self.x_diff_clockwise, self.x_diff_counterclockwise, self.y_diff_clockwise, self.y_diff_counterclockwise, self.nominal_wheel_seperation, self.sqaure_length)
		if self.state_4 == True:
			print("Lee's method calibration factors have been calculated and saved in file Lee_method_calibration_factors.txt")
		else:
			print("Lee's method calibration factors have failed to calculate.")
	
		self.state_5 = self.JungCalibration(self.orientation_diff_clockwise, self.orientation_diff_counterclockwise, self.nominal_wheel_seperation, self.sqaure_length)
		if self.state_5 == True:
			print("Jung's method calibration factors have been calculated and saved in file Jung_method_calibration_factors.txt")
		else:
			print("Jung's method calibration factors have failed to calculate.")


	#define a function for computing the compensation factors of simulation when we want to control the robot running with desired wheel diameter error and wheelbase erro
	def SimulationCompensationFactors(self, sim_Ed, sim_Eb):
		sim_list = []

		sim_right_wheel_factor = 2/((1/sim_Ed)+1)
		sim_left_wheel_factor = 2/(sim_Ed+1)
		sim_wheel_base_factor = sim_Eb
		sim_list.append(sim_right_wheel_factor)
		sim_list.append(sim_left_wheel_factor)
		sim_list.append(sim_wheel_base_factor)

		print("simulation compensation factors for right wheel, left wheel and wheelbase:", sim_right_wheel_factor, sim_left_wheel_factor, sim_wheel_base_factor)
		
		with open("simulation_compensation_factors.txt", 'w') as f:
			for j in sim_list:
				f.write(str(j) + '\n')
		
		return True

	#define a function for computing the calibration factors for robot with kinematic errors(wheel diameter error and wheelbase error)
	#-->new method
	def NewMethodCalibration(self, NM_theta_cw, NM_theta_ccw, NM_bn, NM_L):
		NM_list = []

		alpha = (NM_theta_cw - NM_theta_ccw)/8         # in radians
		beta = (NM_theta_cw + NM_theta_ccw)/8          # in radians
		A = (beta*pi*NM_bn)/(((pi/2)-alpha)*4*NM_L)
		NM_Ed = (1+A)/(1-A)
		NM_Eb = (pi/2)/((pi/2)-alpha)

		NM_right_wheel_factor = 2/((1/NM_Ed)+1)
		NM_left_wheel_factor = 2/(NM_Ed+1)
		NM_wheel_base_factor = NM_Eb
		NM_list.append(NM_right_wheel_factor)
		NM_list.append(NM_left_wheel_factor)
		NM_list.append(NM_wheel_base_factor)
		NM_list.append(NM_Ed)

		print("new method calibration factors for right wheel and left wheel, Eb and Ed:", NM_right_wheel_factor, NM_left_wheel_factor, NM_wheel_base_factor, NM_Ed)
		
		with open("new_method_calibration_factors.txt", 'w') as f:
			for j in NM_list:
				f.write(str(j) + '\n')
		
		return True
	
	#define a function for computing the calibration factors for robot with kinematic errors(wheel diameter error and wheelbase error)
	#-->UMBmark method
	def UMBmarkCalibration(self, UMB_x_cw, UMB_x_ccw, UMB_y_cw, UMB_y_ccw, UMB_bn, UMB_L):
		UMB_list = []

		alpha_1 = (UMB_x_cw + UMB_x_ccw)/(-4*UMB_L)   # in radians
		alpha_2 = (UMB_y_cw - UMB_y_ccw)/(-4*UMB_L)   # in radians
		alpha = (alpha_1 + alpha_2)/2                 # using x and y difference can both calculate the alpha, averaging them for higher precision

		beta_1 = (UMB_x_cw - UMB_x_ccw)/(-4*UMB_L)    # in radians
		beta_2 = (UMB_y_cw + UMB_y_ccw)/(-4*UMB_L)    # in radians
		beta = (beta_1 + beta_2)/2                    # using x and y difference can both calculate the beta, averaging them for higher precision

		R = (UMB_L/2)/(sin(beta/2))
		UMB_Ed = (R + (UMB_bn/2))/(R - (UMB_bn/2))
		UMB_Eb = (pi/2)/((pi/2)-alpha)

		UMB_right_wheel_factor = 2/((1/UMB_Ed)+1)
		UMB_left_wheel_factor = 2/(UMB_Ed+1)
		UMB_wheel_base_factor = UMB_Eb
		UMB_list.append(UMB_right_wheel_factor)
		UMB_list.append(UMB_left_wheel_factor)
		UMB_list.append(UMB_wheel_base_factor)
		UMB_list.append(UMB_Ed)

		print("UMBmark method calibration factors for right wheel and left wheel, Eb and Ed:", UMB_right_wheel_factor, UMB_left_wheel_factor, UMB_wheel_base_factor, UMB_Ed)
		
		with open("UMBmark_method_calibration_factors.txt", 'w') as f:
			for j in UMB_list:
				f.write(str(j) + '\n')
		
		return True

	#define a function for computing the calibration factors for robot with kinematic errors(wheel diameter error and wheelbase error)
	#-->Lee method
	def LeeCalibration(self, Lee_x_cw, Lee_x_ccw, Lee_y_cw, Lee_y_ccw, Lee_bn, Lee_L):
		Lee_list = []

		A = (-4*Lee_L*Lee_L)/(Lee_x_cw - Lee_x_ccw)
		B = (-4*Lee_L*Lee_L)/(Lee_y_cw + Lee_y_ccw)
		Lee_terms_1 = pi*Lee_bn/(4*(A-Lee_bn/2))
		Lee_terms_2 = pi*Lee_bn/(4*(B-Lee_bn/2))

		alpha_1 = (Lee_x_cw + Lee_x_ccw)/(-4*Lee_L) + Lee_terms_1   # in radians
		alpha_2 = (Lee_y_cw - Lee_y_ccw)/(-4*Lee_L) + Lee_terms_2   # in radians
		alpha = (alpha_1 + alpha_2)/2                 # using x and y difference can both calculate the alpha, averaging them for higher precision

		beta_1 = (Lee_x_cw - Lee_x_ccw)/(-4*Lee_L)    # in radians
		beta_2 = (Lee_y_cw + Lee_y_ccw)/(-4*Lee_L)    # in radians
		beta = (beta_1 + beta_2)/2                    # using x and y difference can both calculate the beta, averaging them for higher precision

		R = (Lee_L/2)/(sin(beta/2))
		Lee_Ed = (R + (Lee_bn/2))/(R - (Lee_bn/2))
		Lee_Eb = (pi/2)/((pi/2)-alpha)

		Lee_right_wheel_factor = 2/((1/Lee_Ed)+1)
		Lee_left_wheel_factor = 2/(Lee_Ed+1)
		Lee_wheel_base_factor = Lee_Eb
		Lee_list.append(Lee_right_wheel_factor)
		Lee_list.append(Lee_left_wheel_factor)
		Lee_list.append(Lee_wheel_base_factor)
		Lee_list.append(Lee_Ed)

		print("Lee's method calibration factors for right wheel and left wheel, Eb and Ed:", Lee_right_wheel_factor, Lee_left_wheel_factor, Lee_wheel_base_factor, Lee_Ed)
		
		with open("Lee_method_calibration_factors.txt", 'w') as f:
			for j in Lee_list:
				f.write(str(j) + '\n')
		
		return True

	#define a function for computing the calibration factors for robot with kinematic errors(wheel diameter error and wheelbase error)
	#-->Jung method
	def JungCalibration(self, Jung_theta_cw, Jung_theta_ccw, Jung_bn, Jung_L):
		Jung_list = []

		alpha_1 = (Jung_theta_cw - Jung_theta_ccw)/8         # in radians
		beta = (Jung_theta_cw + Jung_theta_ccw)/8            # in radians
		alpha_2 = (pi*Jung_bn*beta)/(4*Jung_L)               # in radians
		alpha = alpha_1 + alpha_2                            # in radians

		Jung_Eb = (pi/2)/((pi/2)-alpha)

		R = (Jung_L/2)/(sin(beta/2))
		A = (Jung_bn*Jung_Eb)/2

		Jung_Ed = (R + A)/(R - A)

		Jung_right_wheel_factor = 2/((1/Jung_Ed)+1)
		Jung_left_wheel_factor = 2/(Jung_Ed+1)
		Jung_wheel_base_factor = Jung_Eb
		Jung_list.append(Jung_right_wheel_factor)
		Jung_list.append(Jung_left_wheel_factor)
		Jung_list.append(Jung_wheel_base_factor)
		Jung_list.append(Jung_Ed)

		print("Jung's calibration factors for right wheel and left wheel, Eb and Ed:", Jung_right_wheel_factor, Jung_left_wheel_factor, Jung_wheel_base_factor, Jung_Ed)
		
		with open("Jung_method_calibration_factors.txt", 'w') as f:
			for j in Jung_list:
				f.write(str(j) + '\n')
		
		return True


if __name__ == '__main__':
	try:
		dataprocessEbEd()
	except Exception:
		print("Data processing failed")