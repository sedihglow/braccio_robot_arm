import numpy as np

# Handles the kinematic functions definitions and variables
class kinematics:
    # Length in cm
    A0_LEN = 6   # base to shoulder
    A1_LEN = 13  # shoulder to elbow
    A2_LEN = 12  # elbow to wrist vertical
    A3_LEN = 6.5 # wrist vertial to wrist rotation
    A4_LEN = 12  # wrist rotation to end effector

    LINKS = 5      # Links on arm
    DISP_VECTS = 5 # Displacement vectors
    ROT_MATS = 6   # Rotation Matricies
    HOMO_MATS = 6  # Homogeneous Matricies
    DOF_ANGLES = 6 # 6 degrees of freedom

    def __init__(self):

        self.angles = []
        for i in range(0,self.DOF_ANGLES,1):
            self.angles.append(0)
        
        # angle for servo indicies
        self.BASE_M1      = 0 # base
        self.SHOULDER_M2  = 1 # shoulder
        self.ELBOW_M3     = 2 # elbow 
        self.WRIST_VRT_M4 = 3 # wrist vertial
        self.WRIST_ROT_M5 = 4 # wrist rotation
        self.GRIP_M6      = 5 # gripper

        # Length of links between joints (servos) on arm
        self.link_len = []
        for i in range(0,self.LINKS,1):
            self.link_len.append(0)
        
        # link len indicies
        self.A0, self.A1, self.A2, self.A3, self.A4 = [0, 1, 2, 3, 4]

        # length in cm
        self.link_len[self.A0] = self.A0_LEN # base to shoulder
        self.link_len[self.A1] = self.A1_LEN # shoulder to elbow
        self.link_len[self.A2] = self.A2_LEN # elbow to wrist vertical
        self.link_len[self.A3] = self.A3_LEN # wrist vertial to wrist rotation
        self.link_len[self.A4] = self.A4_LEN # wrist rotation to end effector

        # displacement vectors
        self.disp_vec = []
        for i in range(0,self.DISP_VECTS,1):
            self.disp_vec.append(np.array([[0],
                                           [0],
                                           [0]]))
        # initialize displacement vectors
        self.create_fill_disp_vects()

        # rotation matricies 0-1, 1-2, 2-3, etc. up to 5 frames
        self.rot_mat = []
        for i in range(0,self.ROT_MATS,1):
            self.rot_mat.append(np.array([[0],
                                          [0],
                                          [0]]))
        # fills the rotation matricies and provides rot_mat_0_[1-5]
        self.create_rot_matrix()

        self.homo_trans_mat = []
        for i in range(0,self.HOMO_MATS,1):
            self.homo_trans_mat.append(np.array([[0,0,0,0],
                                                 [0,0,0,0],
                                                 [0,0,0,0],
                                                 [0,0,0,0]]))
        self.create_homo_trans()
        

    # set kinematic variables with angles that are set
    def set_kin_vars(self): 
        self.create_rot_matrix()
        self.create_fill_disp_vects()
        self.create_homo_trans()
    
    # creates and fills the homogenous transform matricies, last index is 0_5
    # returns H_0_5 matrix which is index 5 of the homo_trans_mat list
    # NOTE: @ syntax multiplies the matricies
    def create_homo_trans(self):
        for i in range(0,self.HOMO_MATS-1,1):
            axis = 1 # concat matrix vertically
            # combine rot_mat with disp_vect (disp column to the right)
            self.homo_trans_mat[i] = np.concatenate((self.rot_mat[i], 
                                                    self.disp_vec[i]), axis)
            axis = 0 # concat matrix horizontally
            # combine current matrix a new row so we can multiply later
            self.homo_trans_mat[i] = np.concatenate((self.homo_trans_mat[i],
                                                    [[0,0,0,1]]), axis)
        
        # This index holds H_0_5 which contains R_0_5 and D_0_5
        self.homo_trans_mat[5] = (self.homo_trans_mat[0] @ 
                                  self.homo_trans_mat[1] @
                                  self.homo_trans_mat[2] @
                                  self.homo_trans_mat[3] @
                                  self.homo_trans_mat[4])
        return self.homo_trans_mat[5]

    def print_homo_trans_mats(self):
        print("--Homogeneous transform 0_1--")
        print(self.homo_trans_mat[0], "\n")
        print("--Homogeneous transform 1_2--")
        print(self.homo_trans_mat[1], "\n")
        print("--Homogeneous transform 2_3--")
        print(self.homo_trans_mat[2], "\n")
        print("--Homogeneous transform 3_4--")
        print(self.homo_trans_mat[3], "\n")
        print("--Homogeneous transform 4_5--")
        print(self.homo_trans_mat[4], "\n")
        print("--Homogeneous transform 0_5--")
        print(self.homo_trans_mat[5], "\n")

    def print_disp_vects(self):
        print("Displacement Vectors")
        print("---D[0-1]---")
        print(self.disp_vec[0], "\n")
        print("---D[1-2]---")
        print(self.disp_vec[1], "\n")
        print("---D[2-3]---")
        print(self.disp_vec[2], "\n")
        print("---D[3-4]---")
        print(self.disp_vec[3], "\n")
        print("---D[4-5]---")
        print(self.disp_vec[4], "\n")

    # Creates the displacement vectors and places them in self.disp_vec
    def create_fill_disp_vects(self):
        # Convert servo angles from degrees to radians
        a0_rad = np.deg2rad(self.angles[self.BASE_M1])
        a1_rad = np.deg2rad(self.angles[self.SHOULDER_M2])
        a2_rad = np.deg2rad(self.angles[self.ELBOW_M3])
        a3_rad = np.deg2rad(self.angles[self.WRIST_VRT_M4])
        a4_rad = np.deg2rad(self.angles[self.WRIST_ROT_M5])
        a5_rad = np.deg2rad(self.angles[self.GRIP_M6])

        # displacement vector 0-1
        self.disp_vec[0] = np.array([[0],
                                     [0],
                                     [self.link_len[self.A0]]])
        # disp vect 1-2
        self.disp_vec[1] = np.array([[self.link_len[self.A1]*np.cos(a1_rad)],
                                     [self.link_len[self.A1]*np.sin(a1_rad)],
                                     [0]])

        # disp vect 2-3
        self.disp_vec[2] = np.array([[self.link_len[self.A2]*np.cos(a2_rad)],
                                     [self.link_len[self.A2]*np.sin(a2_rad)],
                                     [0]])
        
        # TODO: Finish figuring out disp vect 3-4
        # disp vect 3-4 
        self.disp_vec[3] = np.array([[0],
                                     [0],
                                     [0]])

        # disp vect 4-5
        self.disp_vec[4] = np.array([[0],
                                     [0],
                                     [self.link_len[self.A4]]])

    # returns a numpy array matrix, on error returns zero matrix, finds the 
    # rotation matrix for the angles on the braccio
    # NOTE: @ syntax multiplies the matricies
    def create_rot_matrix(self, start_frame=0, end_frame=5):
        # Convert servo angles from degrees to radians
        a0_rad = np.deg2rad(self.angles[self.BASE_M1])
        a1_rad = np.deg2rad(self.angles[self.SHOULDER_M2])
        a2_rad = np.deg2rad(self.angles[self.ELBOW_M3])
        a3_rad = np.deg2rad(self.angles[self.WRIST_VRT_M4])
        a4_rad = np.deg2rad(self.angles[self.WRIST_ROT_M5])
        a5_rad = np.deg2rad(self.angles[self.GRIP_M6])
        
        if (end_frame < 1 or end_frame > 5 or 
            start_frame < 0 or start_frame > 4 or
            start_frame >= end_frame):
            print("ERROR: Invalid start/end frame for rot matrix")
            error = np.array([[0,0,0],
                              [0,0,0],
                              [0,0,0]])
            return error

        # This matrix helps convert the servo_1 frame to the servo_0 frame.
        self.rot_mat[0] = np.array([[np.cos(a0_rad), 0, np.sin(a0_rad)],
                                    [np.sin(a0_rad), 0, -np.cos(a0_rad)],
                                    [0, 1, 0]])
        
        # This matrix helps convert the servo_2 frame to the servo_1 frame.
        self.rot_mat[1] = np.array([[np.cos(a1_rad), -np.sin(a1_rad), 0],
                                    [np.sin(a1_rad), np.cos(a1_rad), 0],
                                    [0, 0, 1]]) 

        # This matrix helps convert the servo_3 frame to the servo_2 frame.
        self.rot_mat[2] = np.array([[np.cos(a2_rad), -np.sin(a2_rad), 0],
                                    [np.sin(a2_rad), np.cos(a2_rad), 0],
                                    [0, 0, 1]]) 
 
        # This matrix helps convert the servo_4 frame to the servo_3 frame.
        self.rot_mat[3] = np.array([[-np.sin(a3_rad), 0, np.cos(a3_rad)],
                                    [np.cos(a3_rad), 0, np.sin(a3_rad)],
                                    [0, 1, 0]]) 
        
        # This matrix helps convert the servo_5 frame to the servo_4 frame.
        self.rot_mat[4] = np.array([[np.cos(a4_rad), -np.sin(a4_rad), 0],
                                    [np.sin(a4_rad), np.cos(a4_rad), 0],
                                    [0, 0, 1]])
         
        # Calculate the rotation matrix that converts the 
        # end-effector frame (frame 5) to the servo_0 frame. rot_mat_0_5
        self.rot_mat[5] = (self.rot_mat[0] @ self.rot_mat[1] @ self.rot_mat[2] @ 
                           self.rot_mat[3] @ self.rot_mat[4])
        
        if (start_frame == 0):
            if (end_frame == 1):
                return self.rot_mat[0] # rot_mat_0_1
            elif (end_frame == 2):
                rot_mat_0_2 = self.rot_mat[0] @ self.rot_mat[1] 
                return rot_mat_0_2
            elif (end_frame == 3):
                rot_mat_0_3 = (self.rot_mat[0] @ self.rot_mat[1] @ 
                               self.rot_mat[2])
                return rot_mat_0_3
            elif (end_frame == 4):
                rot_mat_0_4 = (self.rot_mat[0] @ self.rot_mat[1] @ 
                               self.rot_mat[2] @ self.rot_mat[3])
                return rot_mat_0_4
            else:
                return self.rot_mat[5]
        elif (start_frame == 1):
            if (end_frame == 2):
                return self.rot_mat[1]
            elif (end_frame == 3):
                rot_mat_1_3 = self.rot_mat[1] @ self.rot_mat[2]
                return rot_mat_1_3
            elif (end_frame == 4):
                rot_mat_1_4 = (self.rot_mat[1] @ self.rot_mat[2] @
                               self.rot_mat[3])
                return rot_mat_1_4
            else:
               rot_mat_1_5 = (self.rot_mat[1] @ self.rot_mat[2] @
                              self.rot_mat[3] @ self.rot_mat[4])
               return rot_mat_1_5
        elif (start_frame == 2):
            if (end_frame == 3):
                return self.rot_mat[2] # rot_mat_2_3
            elif (end_frame == 4):
                rot_mat_2_4 = self.rot_mat[2] @ self.rot_mat[3]
                return rot_mat_2_4
            else:
                rot_mat_2_5 = (self.rot_mat[2] @ self.rot_mat[3] @
                               self.rot_mat[4])
        elif (start_frame == 3):
            if (end_frame == 4):
                return self.rot_mat[3] # rot_mat_3_4
            else:
                rot_mat_3_5 = self.rot_mat[3] @ self.rot_mat[4]
        elif (start_grame == 4):
            return self.rot_mat[4] # rot_mat_4_5

        return self.rot_mat[5] # default return rot_mat_0_5
