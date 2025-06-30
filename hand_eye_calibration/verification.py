import numpy as np

T_base_tool0 = np.array([
    [-0.815, -0.466, -0.345, -0.520],
    [-0.496,  0.868,  0.001,  0.133],
    [ 0.299,  0.172, -0.938,  0.529],
    [ 0.000,  0.000,  0.000,  1.000]
]) # tool position of home pose


#matrix got from hand eye calibration
T_cam2gripper = np.array([
    [0.55626932,  0.82916143, -0.05527892,  0.06101574],
    [-0.83049489, 0.55237473, -0.0718359,   0.04095847],
    [-0.02902888, 0.08586897,  0.99588345,  0.00499622],
    [0.0,         0.0,         0.0,         1.0]
])

T_cam_object = np.array([
    [ 0.48783, -0.8498,   0.19966, -0.16713],
    [ 0.40133,  0.015218, -0.91581,  0.073987],
    [ 0.77522,  0.52689,  0.34847,  0.64577],
    [ 0.0,      0.0,      0.0,      1.0]
])



# Compute object pose in base frame
T_obj2base = T_base_tool0 @ T_cam2gripper @ T_cam_object

# Print result
print("Object pose in base frame:\n", T_obj2base)



