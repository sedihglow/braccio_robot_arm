import numpy as np # Scientific computing library
 
# Project: Calculating Rotation Matrices for a 6 DOF Robotic Arm
# Author: Addison Sears-Collins
# Date created: August 6, 2020
 
# Servo (joint) angles in degrees
servo_0_angle = 0 # Joint 1 base 
servo_1_angle = 0 # Joint 2
servo_2_angle = 0 # Joint 3
servo_3_angle = 0 # Joint 4
servo_4_angle = 0 # Joint 5
 
# This servo would open and close the gripper (end-effector)
servo_5_angle = 0 # Joint 6
 
# Convert servo angles from degrees to radians
servo_0_angle = np.deg2rad(servo_0_angle)
servo_1_angle = np.deg2rad(servo_1_angle)
servo_2_angle = np.deg2rad(servo_2_angle)
servo_3_angle = np.deg2rad(servo_3_angle)
servo_4_angle = np.deg2rad(servo_4_angle)
servo_5_angle = np.deg2rad(servo_5_angle)
 
# This matrix helps convert the servo_1 frame to the servo_0 frame.
rot_mat_0_1 = np.array([[np.cos(servo_0_angle), 0, np.sin(servo_0_angle)],
                        [np.sin(servo_0_angle), 0, -np.cos(servo_0_angle)],
                        [0, 1, 0]]) 
 
# This matrix helps convert the servo_2 frame to the servo_1 frame.
rot_mat_1_2 = np.array([[np.cos(servo_1_angle), -np.sin(servo_1_angle), 0],
                        [np.sin(servo_1_angle), np.cos(servo_1_angle), 0],
                        [0, 0, 1]]) 
 
# This matrix helps convert the servo_3 frame to the servo_2 frame.
rot_mat_2_3 = np.array([[np.cos(servo_2_angle), -np.sin(servo_2_angle), 0],
                        [np.sin(servo_2_angle), np.cos(servo_2_angle), 0],
                        [0, 0, 1]]) 
 
# This matrix helps convert the servo_4 frame to the servo_3 frame.
rot_mat_3_4 = np.array([[-np.sin(servo_3_angle), 0, np.cos(servo_3_angle)],
                        [np.cos(servo_3_angle), 0, np.sin(servo_3_angle)],
                        [0, 1, 0]]) 
 
# This matrix helps convert the servo_5 frame to the servo_4 frame.
rot_mat_4_5 = np.array([[np.cos(servo_4_angle), -np.sin(servo_4_angle), 0],
                        [np.sin(servo_4_angle), np.cos(servo_4_angle), 0],
                        [0, 0, 1]]) 
 
# Calculate the rotation matrix that converts the 
# end-effector frame (frame 5) to the servo_0 frame.
rot_mat_0_5 = rot_mat_0_1 @ rot_mat_1_2 @ rot_mat_2_3 @ rot_mat_3_4 @ rot_mat_4_5
 
# Display the rotation matrix
print(rot_mat_0_5)
