import numpy as np
import cv2 as cv
import glob
import pickle
import os


################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

chessboardSize = (9,6)
frameSize = (1280,720)



# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 80
objp = objp * size_of_chessboard_squares_mm


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.



# Path to the folder containing images
folder_path="C:\\Users\\BYNDLA KARTHIK KUMAR\\Downloads\\new_calibration\\images"

# List to store the images
images = []

# Iterate over each file in the folder
for filename in os.listdir(folder_path):
    # Check if the file is an image file
    if filename.endswith(".jpg") or filename.endswith(".png"):
        # Read the image and append it to the list
        image_path = os.path.join(folder_path, filename)
        img = cv.imread(image_path)

        if img is not None:
            images.append(img)
        else:
            print(f"Unable to read {filename}")
           

# Now 'images' contains all the images from the folder as NumPy arrays
    
for img in images:
 
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)
   
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(1000)


cv.destroyAllWindows()




ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

# Save the camera calibration result for later use (we won't worry about rvecs / tvecs)
pickle.dump((cameraMatrix, dist), open( "calibration.pkl", "wb" ))
pickle.dump(cameraMatrix, open( "cameraMatrix.pkl", "wb" ))
pickle.dump(dist, open( "dist.pkl", "wb" ))
print("cameramatrix",cameraMatrix)
print("distortion coefficients",dist)


############## UNDISTORTION #####################################################

img = cv.imread("C:\\Users\\BYNDLA KARTHIK KUMAR\\OneDrive\\Pictures\\Camera Roll\\chessboard.jpg")
h,  w = img.shape[:2]
newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))



# Undistort
dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

x, y, w, h = roi

dst = dst[y:y+h, x:x+w]

cv.imwrite('caliResult1.png', img)


# Reprojection Error
mean_error = 0

for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error

print( "total error: {}".format(mean_error/len(objpoints)) )

# np.save('sampf.npy',np.array([cameraMatrix,rvecs,tvecs,dist,newCameraMatrix]))
np.save('samp1.npy', cameraMatrix)
np.save('samp2.npy', rvecs)
np.save('samp3.npy', tvecs)
np.save('samp4.npy', dist)
np.save('samp5.npy', newCameraMatrix)