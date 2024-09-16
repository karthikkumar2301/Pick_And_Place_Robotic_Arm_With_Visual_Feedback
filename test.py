import numpy as np
import cv2 as cv
from PIL import Image

from util import get_limits

from util import get_limits_1

from util import get_limits_2
rotation_matrix  =np.load('sampr.npy')
translation_vector =np.load('sampt.npy')
cameraMatrix = np.load('samp1.npy')
rvecs = np.load('samp2.npy')
tvecs = np.load('samp3.npy')
dist = np.load('samp4.npy')
newCameraMatrix = np.load('samp5.npy')
x2=h1=w1=y2=x1=y1=w=h=0

yellow = [0, 255, 255]  # yellow in RGB colorspace
blue = [255, 0, 0]  # blue in RGB colorspace

cap = cv.VideoCapture(0)
frame_width = 1280
frame_height = 720

cap.set(cv.CAP_PROP_FRAME_WIDTH, frame_width)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, frame_height)

while True:
    ret, frame = cap.read()
    h,  w = frame.shape[:2]
    newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))
    cv.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
    frame = cv.GaussianBlur(frame, (3, 3), 0)
    
    hsvImage = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # Color segmentation for yellow
    lowerLimit, upperLimit = get_limits(color=yellow)
    mask = cv.inRange(hsvImage, lowerLimit, upperLimit)
    
    # Color segmentation for blue
    lowerLimit, upperLimit = get_limits_1(color=blue)
    maskB = cv.inRange(hsvImage, lowerLimit, upperLimit)
    
    
    # Find contours for yellow objects
    contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv.contourArea(cnt) > 200:
            x1, y1, w, h = cv.boundingRect(cnt)
            cv.rectangle(frame, (x1, y1), (x1+w, y1+h), (0, 255, 0), 5)
            cv.putText(frame, 'Yellow', (x1+w//2, y1+h//2), cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1, cv.LINE_8)
    
    # Find contours for blue objects
    contours, _ = cv.findContours(maskB, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv.contourArea(cnt) > 200:
            x2, y2, w1, h1 = cv.boundingRect(cnt)
            cv.rectangle(frame, (x2, y2), (x2+w1, y2+h1), (0, 255, 0), 5)
            cv.putText(frame, 'Blue', (x2+w1//2, y2+h1//2), cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1, cv.LINE_8)
    

    
    cv.imshow('frame', frame)

    pixel_y = (y1 + h//2)
    pixel_x = (x1 + w//2)

    pixel_coordinates=np.array([pixel_x,pixel_y,1])
    undistorted_point = np.array([[pixel_x,pixel_y, 1]], dtype=np.float32)

    normalized_point = np.dot(np.linalg.inv(cameraMatrix), undistorted_point.T)
    normalized_point /= normalized_point[2]
# Convert camera coordinates to world coordinates (assuming Z-coordinate is known)
    scaling_factor=1.795
    z=54
    Z = z*scaling_factor
    camera_coords = np.dot(np.linalg.inv(cameraMatrix),pixel_coordinates)*Z

# Print the caplculated world coordinatesq


# Print the calculated world coordinates
    k=cv.waitKey(1)
    if k & 0xFF==ord('p'):
        #print(world_coords)
        print(camera_coords)
        print(x1+w//2,y1+h//2)
    
    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()

