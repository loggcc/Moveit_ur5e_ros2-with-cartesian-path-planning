import cv2
import numpy as np
import math

def R_T_to_homogeneous_matrix(R, T):
    """
    Convert rotation matrix and translation vector to homogeneous matrix
    """
    homo_matrix = np.eye(4)
    homo_matrix[:3, :3] = R
    homo_matrix[:3, 3] = T.flatten()
    return homo_matrix

def homogeneous_matrix_to_RT(homo_matrix):
    """
    Decompose homogeneous matrix into rotation matrix and translation vector
    """
    R = homo_matrix[:3, :3]
    T = homo_matrix[:3, 3].reshape(3, 1)
    return R, T

def is_rotation_matrix(R):
    """
    Check if matrix is a valid rotation matrix
    """
    if R.shape[0] >= 3 and R.shape[1] >= 3:
        R_33 = R[:3, :3]
    else:
        R_33 = R
    
    Rt = np.transpose(R_33)
    should_be_identity = np.dot(Rt, R_33)
    I = np.identity(3, dtype=R_33.dtype)
    
    return np.linalg.norm(I - should_be_identity) < 1e-6

def euler_angle_to_rotation_matrix(euler_angle, seq):
    """
    Convert Euler angles to rotation matrix - FIXED VERSION
    """
    assert euler_angle.size == 3, "Euler angle must have 3 elements"
    
    # Convert degrees to radians
    euler_rad = np.radians(euler_angle.flatten())
    
    rx, ry, rz = euler_rad[0], euler_rad[1], euler_rad[2]
    rxs, rxc = np.sin(rx), np.cos(rx)
    rys, ryc = np.sin(ry), np.cos(ry)
    rzs, rzc = np.sin(rz), np.cos(rz)
    
    # Rotation matrices for X, Y, Z axes
    RotX = np.array([[1, 0, 0],
                     [0, rxc, -rxs],
                     [0, rxs, rxc]])
    
    RotY = np.array([[ryc, 0, rys],
                     [0, 1, 0],
                     [-rys, 0, ryc]])
    
    RotZ = np.array([[rzc, -rzs, 0],
                     [rzs, rzc, 0],
                     [0, 0, 1]])
    
    # FIXED: Correct composition order
    if seq == "zyx":
        rot_mat = np.dot(np.dot(RotZ, RotY), RotX)  # Z*Y*X (intrinsic)
    elif seq == "xyz":
        rot_mat = np.dot(np.dot(RotX, RotY), RotZ)  # X*Y*Z (intrinsic)
    elif seq == "yzx":
        rot_mat = np.dot(np.dot(RotY, RotZ), RotX)
    elif seq == "zxy":
        rot_mat = np.dot(np.dot(RotZ, RotX), RotY)
    elif seq == "yxz":
        rot_mat = np.dot(np.dot(RotY, RotX), RotZ)
    elif seq == "xzy":
        rot_mat = np.dot(np.dot(RotX, RotZ), RotY)
    else:
        raise ValueError("Euler Angle Sequence string is wrong...")
    
    if not is_rotation_matrix(rot_mat):
        raise ValueError("Euler Angle convert to Rotation Matrix failed...")
    
    return rot_mat

def attitude_vector_to_matrix(m, use_quaternion=False, seq=""):
    """
    Convert raw pose data to homogeneous matrix
    """
    m = np.array(m).flatten()
    assert len(m) == 6 or len(m) == 10, "Input must have 6 or 10 elements"
    
    temp = np.eye(4)
    
    if use_quaternion:
        quaternion_vec = m[3:7]
        temp[:3, :3] = quaternion_to_rotation_matrix(quaternion_vec)
    else:
        if len(m) == 6:
            rot_vec = m[3:6]
        elif len(m) == 10:
            rot_vec = m[7:10]
        
        if seq == "":  # Rotation vector (Rodrigues)
            rot_matrix, _ = cv2.Rodrigues(rot_vec)
            temp[:3, :3] = rot_matrix
        else:  # Euler angles
            temp[:3, :3] = euler_angle_to_rotation_matrix(rot_vec, seq)
    
    temp[:3, 3] = m[:3]
    return temp

def main():
    # Your B_matrices (camera poses) - keeping unchanged
    B_matrices = [
            np.array([[-0.98437641, -0.16985841, -0.04638116,  0.21136383],
                    [ 0.15910901, -0.97092231,  0.17886921,  0.11534585],
                    [-0.07541494,  0.16869497,  0.98277902,  0.48221166],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                    
            np.array( [[-0.99681779,  0.07056337, -0.03708247,  0.13325405],
                    [-0.05982646, -0.9696736 , -0.23696817,  0.29772657],
                    [-0.05267916, -0.23399557,  0.97080945,  0.56352987],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]),

            np.array( [[ 0.73163281,  0.64259513,  0.22756302, -0.30025999],
                    [-0.68125261,  0.67713558,  0.27817673,  0.07963977],
                    [ 0.024664  , -0.35855112,  0.93318422,  0.57460235],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]),
            
            np.array([[ 0.75521946,  0.59407722,  0.27697622,  0.00538862],
                    [-0.64412083,  0.59432883,  0.48154086,  0.12811089],
                    [ 0.1214575 , -0.54207518,  0.83150621,  0.50640572],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]),
               
            np.array( [[-0.97602365,  0.21451844,  0.03687375, -0.02034627],
                    [-0.21760972, -0.95787407, -0.18741206,  0.02500527],
                    [-0.00488293, -0.19094269,  0.98158904,  0.64198114],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]),


            np.array([[-0.97030607,  0.18533128, -0.15542989, -0.03967454],
                    [-0.20844764, -0.96667218,  0.14864209,  0.06704947],
                    [-0.12270173,  0.17662731,  0.97659975,  0.48265465],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]),


            np.array(
                    [[-0.77413675,  0.60095659, -0.19890569,  0.12882126],
                    [-0.63292195, -0.7293306 ,  0.25978199,  0.16033433],
                    [ 0.01104969,  0.32699856,  0.94496023,  0.39465784],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]),

            np.array(    [[-0.77845633,  0.60785226,  0.156593  , -0.02157804],
                    [-0.60627662, -0.79273414,  0.06325545,  0.27822214],
                    [ 0.16258658, -0.04569707,  0.98563552,  0.62650351],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]),

            np.array([[-0.76682969,  0.58640017,  0.26097332, -0.0131195 ],
                    [-0.60652674, -0.7950516 ,  0.00427507,  0.25760088],
                    [ 0.20999416, -0.15500905,  0.96533655,  0.52305745],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]),
            np.array([[-0.89389376, -0.29920372, -0.33381294,  0.17267122],
                    [ 0.40705639, -0.85368401, -0.32485182,  0.12752076],
                    [-0.1877739 , -0.4262637 ,  0.8848956 ,  0.60236719],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]),
            np.array([[-0.72920739, -0.31887147, -0.60545649, -0.0820753 ],
                    [ 0.54363478, -0.807317  , -0.22956586,  0.08377625],
                    [-0.41559331, -0.49654833,  0.76205115,  0.72100566],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]),
            np.array(  [[-0.18977172, -0.55973023, -0.80665282, -0.0513334 ],
                    [ 0.95477903, -0.29672617, -0.0187236 , -0.03313362],
                    [-0.22887484, -0.77372841,  0.59072892,  0.82058363],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]),
       
                    ]

    # Robot tool poses (x,y,z,rx,ry,rz) - degrees for rotation


    T_matrices = np.array([
        [[-0.927, -0.365,  0.089, -0.491],
        [-0.371,  0.927, -0.061,  0.151],
        [-0.060, -0.090, -0.994,  0.519],
        [ 0.000,  0.000,  0.000,  1.000]],

        [[-0.778, -0.539, -0.324, -0.153],
        [-0.565,  0.825, -0.016,  0.107],
        [ 0.276,  0.170, -0.946,  0.476],
        [ 0.000,  0.000,  0.000,  1.000]],

        [[ 0.928, -0.185, -0.324, -0.330],
        [-0.199, -0.980, -0.011,  0.226],
        [-0.315,  0.075, -0.946,  0.537],
        [ 0.000,  0.000,  0.000,  1.000]],

        [[ 0.846, -0.184, -0.500, -0.505],
        [-0.156, -0.983,  0.098, -0.014],
        [-0.510, -0.005, -0.860,  0.546],
        [ 0.000,  0.000,  0.000,  1.000]],

        [[-0.694, -0.659, -0.291, -0.437],
        [-0.682,  0.731, -0.028, -0.037],
        [ 0.231,  0.179, -0.956,  0.649],
        [ 0.000,  0.000,  0.000,  1.000]],

        [[-0.744, -0.662,  0.091, -0.608],
        [-0.667,  0.730, -0.148,  0.004],
        [ 0.031, -0.171, -0.985,  0.520],
        [ 0.000,  0.000,  0.000,  1.000]],

        [[-0.339, -0.907,  0.251, -0.687],
        [-0.940,  0.334, -0.064,  0.207],
        [-0.026, -0.257, -0.966,  0.424],
        [ 0.000,  0.000,  0.000,  1.000]],

        [[-0.358, -0.927, -0.116, -0.329],
        [-0.931,  0.344,  0.123,  0.024],
        [-0.074,  0.152, -0.986,  0.666],
        [ 0.000,  0.000,  0.000,  1.000]],

        [[-0.367, -0.903, -0.224, -0.299],
        [-0.926,  0.332,  0.178,  0.009],
        [-0.087,  0.273, -0.958,  0.527],
        [ 0.000,  0.000,  0.000,  1.000]],

        [[-0.847, -0.134, -0.514, -0.149],
        [-0.093,  0.990, -0.105,  0.111],
        [ 0.523, -0.041, -0.851,  0.420],
        [ 0.000,  0.000,  0.000,  1.000]],

        [[-0.786, -0.106, -0.609, -0.164],
        [ 0.116,  0.942, -0.314,  0.074],
        [ 0.607, -0.318, -0.728,  0.570],
        [ 0.000,  0.000,  0.000,  1.000]],

        [[-0.551,  0.375, -0.745, -0.081],
        [ 0.719,  0.666, -0.197,  0.122],
        [ 0.423, -0.644, -0.637,  0.560],
        [ 0.000,  0.000,  0.000,  1.000]]

       
    ])

    # Data preparation
    R_gripper2base = []
    T_gripper2base = []
    R_target2cam = []
    T_target2cam = []
    homo_target2cam = []
    homo_gripper2base = []

    # Extract from B_matrices (target to camera transforms)
    for mat in B_matrices:
        R = mat[:3, :3]
        T = mat[:3, 3].reshape(3, 1)
        rvec, _ = cv2.Rodrigues(R)
        R_target2cam.append(rvec.flatten())
        T_target2cam.append(T)
        homo_target2cam.append(mat)

    # Extract from T_matrices (gripper to base transforms)
    for mat in T_matrices:
        R = mat[:3, :3]
        T = mat[:3, 3].reshape(3, 1)
        R_gripper2base.append(R)
        T_gripper2base.append(T)
        homo_gripper2base.append(mat)

    # Calibration and error calculation
    methods = [
        (cv2.CALIB_HAND_EYE_TSAI, "TSAI"),
        (cv2.CALIB_HAND_EYE_PARK, "PARK"),
        (cv2.CALIB_HAND_EYE_HORAUD, "HORAUD"),
        (cv2.CALIB_HAND_EYE_ANDREFF, "ANDREFF"),
        (cv2.CALIB_HAND_EYE_DANIILIDIS, "DANIILIDIS")
    ]

    print("\n=== Running Hand-Eye Calibration with All Methods ===")

    for method, name in methods:
        try:
            R_cam2gripper, T_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base, T_gripper2base,
                R_target2cam, T_target2cam,
                method=method
            )
            homo_cam2gripper = R_T_to_homogeneous_matrix(R_cam2gripper, T_cam2gripper)
            print(f"{name} method: Hand-eye transformation matrix (cam to gripper):\n{homo_cam2gripper}")

            positions = []
            for i in range(len(homo_gripper2base)):
                chess_pos = np.array([0.0, 0.0, 0.0, 1.0]).reshape(4, 1)
                world_pos = homo_gripper2base[i] @ homo_cam2gripper @ homo_target2cam[i] @ chess_pos
                positions.append(world_pos[:3].flatten())

            positions = np.array(positions)
            std_error = np.std(positions, axis=0)
            total_error = np.linalg.norm(std_error)

            print(f"{name} method: Total error = {total_error:.4f}")
            print(f"  Position std (x, y, z): {std_error}")

        except Exception as e:
            print(f"{name} method failed: {e}")
if __name__ == "__main__":
    main()
