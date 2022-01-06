import numpy as np

class kinematics:
    A0_LEN = 6   # base to shoulder
    A1_LEN = 13  # shoulder to elbow
    A2_LEN = 12  # elbow to wrist vertical
    A3_LEN = 6.5 # wrist vertial to wrist rotation
    A4_LEN = 12  # wrist rotation to end effector

    def __init__(self):

        self.angles = []
        for i in range(0,6,1):
            self.angles.append(0)

        # Length of links between joints (servos) on arm
        self.link_len = []
        for i in range(0,5,1):
            self.link_len.append(0)
        
        # length in cm
        self.link_len[0] = self.A0_LEN # base to shoulder
        self.link_len[1] = self.A1_LEN # shoulder to elbow
        self.link_len[2] = self.A2_LEN # elbow to wrist vertical
        self.link_len[3] = self.A3_LEN # wrist vertial to wrist rotation
        self.link_len[4] = self.A4_LEN # wrist rotation to end effector

        # displacement vectors
        self.disp_vec = []
        for i in range(0,5,1):
            self.disp_vec.append(np.array([[0],
                                           [0],
                                           [0]]))
        # initialize displacement vectors
        self.create_fill_disp_vects()

    def print_disp_vects(self):
        print("Displacement Vectors")
        print("---D[0-1]---")
        print(self.disp_vec[0])
        print("---D[1-2]---")
        print(self.disp_vec[1])
        print("---D[2-3]---")
        print(self.disp_vec[2])
        print("---D[3-4]---")
        print(self.disp_vec[3])
        print("---D[4-5]---")
        print(self.disp_vec[4])

    # Creates the displacement vectors and places them in self.disp_vec
    def create_fill_disp_vects(self):
        # Convert servo angles from degrees to radians
        a0_rad = np.deg2rad(self.angles[0])
        a1_rad = np.deg2rad(self.angles[1])
        a2_rad = np.deg2rad(self.angles[2])
        a3_rad = np.deg2rad(self.angles[3])
        a4_rad = np.deg2rad(self.angles[4])
        a5_rad = np.deg2rad(self.angles[5])

        # displacement vector 0-1
        self.disp_vec[0] = np.array([[0],
                                     [0],
                                     [self.link_len[0]]])
        # disp vect 1-2
        self.disp_vec[1] = np.array([[self.link_len[1]*np.cos(a1_rad)],
                                     [self.link_len[1]*np.sin(a1_rad)],
                                     [0]])

        # disp vect 2-3
        self.disp_vec[2] = np.array([[self.link_len[2]*np.cos(a2_rad)],
                                     [self.link_len[2]*np.sin(a2_rad)],
                                     [0]])
        
        # TODO: Finish figuring out disp vect 3-4
        # disp vect 3-4 
        self.disp_vec[3] = np.array([[0],
                                     [0],
                                     [0]])

        # disp vect 4-5
        self.disp_vec[4] = np.array([[0],
                                     [0],
                                     [self.link_len[4]]])

    # returns a numpy array matrix, on error returns zero matrix, finds the 
    # rotation matrix for the angles on the braccio
    def create_rot_matrix(self, end_frame):
        # Convert servo angles from degrees to radians
        a0_rad = np.deg2rad(self.angles[0])
        a1_rad = np.deg2rad(self.angles[1])
        a2_rad = np.deg2rad(self.angles[2])
        a3_rad = np.deg2rad(self.angles[3])
        a4_rad = np.deg2rad(self.angles[4])
        a5_rad = np.deg2rad(self.angles[5])
        
        if (end_frame == 0):
            print("ERROR: Invalid start and end frame rot mat, same value")
            error = np.array([[0,0,0],
                              [0,0,0],
                              [0,0,0]])
            return error

        # This matrix helps convert the servo_1 frame to the servo_0 frame.
        rot_mat_0_1 = np.array([[np.cos(a0_rad), 0, np.sin(a0_rad)],
                                [np.sin(a0_rad), 0, -np.cos(a0_rad)],
                                [0, 1, 0]])
        
        if (end_frame == 1):
            return rot_mat_0_1

        # This matrix helps convert the servo_2 frame to the servo_1 frame.
        rot_mat_1_2 = np.array([[np.cos(a1_rad), -np.sin(a1_rad), 0],
                                [np.sin(a1_rad), np.cos(a1_rad), 0],
                                [0, 0, 1]]) 
        if (end_frame == 2):
            rot_mat_0_2 = rot_mat_0_1 @ rot_mat_1_2
            return rot_mat_0_2

        # This matrix helps convert the servo_3 frame to the servo_2 frame.
        rot_mat_2_3 = np.array([[np.cos(a2_rad), -np.sin(a2_rad), 0],
                                [np.sin(a2_rad), np.cos(a2_rad), 0],
                                [0, 0, 1]]) 
 
        if (end_frame == 3):
            rot_mat_0_3 = rot_mat_0_1 @ rot_mat_1_2 @ rot_mat_2_3
            return rot_mat_0_3

        # This matrix helps convert the servo_4 frame to the servo_3 frame.
        rot_mat_3_4 = np.array([[-np.sin(a3_rad), 0, np.cos(a3_rad)],
                                [np.cos(a3_rad), 0, np.sin(a3_rad)],
                                [0, 1, 0]]) 
        
        if (end_frame == 4):
            rot_mat_0_4 = (rot_mat_0_1 @ rot_mat_1_2 @ rot_mat_2_3 @
                           rot_mat_3_4)
            return rot_mat_0_4

        # This matrix helps convert the servo_5 frame to the servo_4 frame.
        rot_mat_4_5 = np.array([[np.cos(a4_rad), -np.sin(a4_rad), 0],
                                [np.sin(a4_rad), np.cos(a4_rad), 0],
                                [0, 0, 1]])
         
        # Calculate the rotation matrix that converts the 
        # end-effector frame (frame 5) to the servo_0 frame.
        rot_mat_0_5 = (rot_mat_0_1 @ rot_mat_1_2 @ rot_mat_2_3 @ rot_mat_3_4 @ 
                       rot_mat_4_5)
        return rot_mat_0_5

 
