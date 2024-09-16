import numpy as np
import cv2 as cv
import serial
import time
import math
from PIL import Image

from util import get_limits

from util import get_limits_1

a1=8.3
a2=20
yy1=2.55
yy2=2.4
max_limit=24.5
min_limit=16.5
time.sleep(1)
arduino = serial.Serial(port='COM7', baudrate=9600, timeout=.1)
def write_read(x,y):
    data_to_send = f"{x}\n"

    arduino.write(data_to_send.encode())
    time.sleep(0.01)
    data_to_send = f"{y}\n"
    arduino.write(data_to_send.encode())
    time.sleep(0.01)
    return

def place(x):
    data_to_send = f"{x}\n"
    arduino.write(data_to_send.encode())
    time.sleep(0.01)
    return

rotation_matrix  =np.load('sampr.npy')
translation_vector =np.load('sampt.npy')
cameraMatrix = np.load('samp1.npy')
rvecs = np.load('samp2.npy')
tvecs = np.load('samp3.npy')
dist = np.load('samp4.npy')
newCameraMatrix = np.load('samp5.npy')
x2=h1=w1=y2=x1=y1=x3=y3=w=h=w2=h2=total_xyz=last_objects=min_index=min_xyz=start=0

Z = 54*1.795
yellow = [0, 255, 255]  # yellow in RGB colorspace
blue = [255, 0, 0]  # blue in RGB colorspace
green = [0, 255, 0]  # green in RGB colorspace

cap = cv.VideoCapture(1)
frame_width = 1280
frame_height = 720
max_width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
max_height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
cap.set(cv.CAP_PROP_FRAME_WIDTH, frame_width)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, frame_height)
print("place the object")
time.sleep(10)
while True:
    print("place the object")
    time.sleep(3)
    ret, frame = cap.read()
    h,  w = frame.shape[:2]
    newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))
    cv.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
    frame = cv.GaussianBlur(frame, (5, 5), 0)
    
    hsvImage = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # Color segmentation for yellow
    lowerLimit, upperLimit = get_limits(color=yellow)
    mask = cv.inRange(hsvImage, lowerLimit, upperLimit)
    
    # Color segmentation for blue
    lowerLimit, upperLimit = get_limits_1(color=blue)
    maskB = cv.inRange(hsvImage, lowerLimit, upperLimit)
   
    # Find contours for yellow objects
    contours_y, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    area=0
    number_of_yellow=0
    for contour in contours_y:
    # Calculate the area of the contour
        area = cv.contourArea(contour)
    # If the area of the contour is greater than 200
        if area > 500:
        # Increment the count of contours with area greater than 200
            number_of_yellow += 1
    yellow_uv = np.empty((number_of_yellow,2), dtype=np.float32)
    yellow_xyz= np.empty((number_of_yellow,3),dtype=np.float32)
    temp=0 
    for cnt in contours_y:
        if cv.contourArea(cnt) > 500:
            x1, y1, w, h = cv.boundingRect(cnt)
            cv.rectangle(frame, (x1, y1), (x1+w, y1+h), (0, 255, 0), 5)
            cv.putText(frame, 'Yellow', (x1+w//2, y1+h//2), cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1, cv.LINE_8)
            pixel_y = (y1 + h//2)
            pixel_x = (x1 + w//2)
            pixel_coordinates=np.array([pixel_x,pixel_y,1])
            yellow_uv[temp,:]=[x1+w//2,y1+h//2]
            world_coords = np.dot(np.linalg.inv(cameraMatrix),pixel_coordinates)*Z
            uv_1=np.array([[pixel_x,pixel_y,1]], dtype=np.float32)
            uv_1=uv_1.T
            suv_1=Z*uv_1
            xyz_c=np.linalg.inv(cameraMatrix).dot(suv_1)
            xyz_c=xyz_c-translation_vector
            XYZ=np.linalg.inv(rotation_matrix).dot(xyz_c)
            XYZ[2,0]=-1
            yellow_xyz[temp,:]=XYZ.T
            temp=temp+1
            
    
    # Find contours for blue objects
    contours_b, _ = cv.findContours(maskB, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    area=0
    number_of_blue=0
    for contour in contours_b:
    # Calculate the area of the contour
        area = cv.contourArea(contour)
    
    # If the area of the contour is greater than 200
        if area > 500:
        # Increment the count of contours with area greater than 200
            number_of_blue += 1
    blue_uv = np.empty((number_of_blue,2), dtype=np.float32)
    blue_xyz= np.empty((number_of_blue,3),dtype=np.float32)
    temp=0  
    for cnt in contours_b:
        if cv.contourArea(cnt) > 500:
            x2, y2, w1, h1 = cv.boundingRect(cnt)
            cv.rectangle(frame, (x2, y2), (x2+w1, y2+h1), (0, 255, 0), 5)
            cv.putText(frame, 'Blue', (x2+w1//2, y2+h1//2), cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1, cv.LINE_8)
            blue_uv[temp,:]=[x2+w1//2,y2+h1//2]
            pixel_y = (y2 + h1//2)
            pixel_x = (x2 + w1//2)
            pixel_coordinates=np.array([pixel_x,pixel_y,1])
            world_coords = np.dot(np.linalg.inv(cameraMatrix),pixel_coordinates)*Z
            uv_1=np.array([[pixel_x,pixel_y,1]], dtype=np.float32)
            uv_1=uv_1.T
            suv_1=Z*uv_1
            xyz_c=np.linalg.inv(cameraMatrix).dot(suv_1)
            xyz_c=xyz_c-translation_vector
            XYZ=np.linalg.inv(rotation_matrix).dot(xyz_c)
            XYZ[2,0]=-2
            blue_xyz[temp,:]=XYZ.T
            temp=temp+1
    
    

# Define your constraints
    
     # List of tuples containing multiple x constraints
    constraints_y = [(-10, 0), (-10, 0)]  # List of tuples containing multiple y constraints

# Create boolean masks for each constraint for x and y
    mask_y = np.zeros(len(yellow_xyz), dtype=bool)

    for constraint in constraints_y:
        mask_y |= (yellow_xyz[:, 1] >= constraint[0]) & (yellow_xyz[:, 1] <= constraint[1])

# Combine the masks using logical OR
    mask = mask_y

# Select rows that don't satisfy the constraints
    filtered_data_yellow = yellow_xyz[~mask]

    print("yellow_xyz:",filtered_data_yellow)
    number_of_yellow=len(filtered_data_yellow)
    
    mask_y = np.zeros(len(blue_xyz), dtype=bool)

    for constraint in constraints_y:
        mask_y |= (blue_xyz[:, 1] >= constraint[0]) & (blue_xyz[:, 1] <= constraint[1])

# Combine the masks using logical OR
    mask = mask_y

# Select rows that don't satisfy the constraints
    filtered_data_blue = blue_xyz[~mask]

    print("blue_xyz:",filtered_data_blue)
    number_of_blue=len(filtered_data_blue)
    

    
    
    current_objects=number_of_blue+number_of_yellow
    print("current_objects:",current_objects)
    total_xyz= np.concatenate((filtered_data_yellow,filtered_data_blue),axis=0)
    xy_points = total_xyz[:, :2]
    norms = np.linalg.norm(xy_points, axis=1)

    # Step 2: Sort points based on norms
    sorted_indices = np.argsort(norms)

    # Step 3: Re-arrange array
    sorted_points =total_xyz[sorted_indices]
    if current_objects>0 :
        min_xyz = sorted_points[0]
        for i in sorted_points :
            check=math.sqrt(i[0]*i[0]+(i[1]+yy1+yy2)*(i[1]+yy1+yy2))-yy2
            if check>min_limit and check<max_limit and math.sqrt(check*check)<a1+a2:
                min_xyz=i
                break
                
    else:
        min_xyz=np.zeros((1,3))
    
    
    
    
    if last_objects>current_objects and start>0 :
        print("picked up succesfully")
    
    data=0
    if min_xyz.all()==0: 
        print("No objects detected.")
        x=0
        y=0
        z=0
    else:
        if start>0 and last_objects==current_objects  :
            print("failed to pick will pick again")
        x = np.round(min_xyz[0], 2) 
        y = np.round(min_xyz[1], 2) 
        z = 0
        print(x,y,z)
 

    check=math.sqrt(x*x+(y+yy1+yy2)*(y+yy1+yy2))-yy2
    if check>min_limit and check<max_limit and math.sqrt(check*check+z*z)<a1+a2:
        write_read(x,y)
        if min_xyz[2]==-1 :
            place(1)
        else:
            place(2)
            

        while(data==0 or data==""):
            data=arduino.readline().strip().decode()
    elif min_xyz.any()!=0:
        print("out of range")
    start=1
    last_objects=current_objects
    cv.imshow('frame', frame)

    
    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
