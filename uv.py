import cv2
import numpy as np
cameraMatrix = np.load('samp1.npy')
rvecs = np.load('samp2.npy')
tvecs = np.load('samp3.npy')
dist = np.load('samp4.npy')
# Define 3D object points
object_points = np.array([[0, 0, 0],        
                           [9, 0, 0],        
                           [18, 0, 0],        
                           [27, 0, 0],       
                           [ 0, 9, 0],
                           [ 0,18,0 ],
                           [ 0,-9,0],
                           [-9,0,0],
                           [-18,0,0],
                            [-27,0,0],
                            [9,9,0],
                            [18,9,0],
                            [27,9,0],
                            [-9,9,0],
                            [-18,9,0],
                            [-27,9,0],
                            [9,18,0],
                            [18,18,0],
                            [27,18,0],
                            [-9,18,0],
                            [-18,18,0],
                            [-27,18,0],
                            [9,-9,0],
                            [18,-9,0],
                            [27,-9,0],
                            [-9,-9,0],
                            [-18,-9,0],
                            [-27,-9,0]],
                          dtype=np.float32)

# Define corresponding 2D image points
image_points = np.array([[678, 230],   
                          [500, 231],   
                          [319, 234],   
                          [137, 233],  
                          [679,410],
                          [684,591],
                          [681,45],
                          [863,223],
                          [1044,227],
                          [1232,224],
                          [503,408],
                          [321,412],
                          [136,411],
                          [861,406],
                          [1054,404],
                          [1232,402],
                          [503,589],
                          [322,591],
                          [140,585],
                          [863,587],
                          [1043,586],
                          [1234,580],
                          [490,42],
                          [317,36],
                          [137,41],
                          [871,43],
                          [1050,39],
                          [1235,37]],
                         dtype=np.float32)

# Define camera intrinsic matrix (camera matrix)
camera_matrix = cameraMatrix

# Define distortion coefficients
dist_coeffs = dist

# Solve PnP problem
success, rotation_vector, translation_vector = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

# Convert rotation vector to rotation matrix
rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

# Print results
print("Rotation Vector:\n", rotation_vector)
print("Translation Vector:\n", translation_vector)
print("Rotation Matrix:\n", rotation_matrix)
np.save('sampr.npy',rotation_matrix)
np.save('sampt.npy', translation_vector)
