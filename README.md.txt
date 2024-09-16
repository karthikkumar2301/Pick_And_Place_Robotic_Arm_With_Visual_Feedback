                                                           PICK AND PLACE ROBOTIC ARM WITH VISUAL FEEDBACK
                                                           -----------------------------------------------


 **Requirements** : Make sure that you have installed python and arduino in your PC

 Modules to be installed in python : Open-cv, Numpy, Serial, pillow. If you get any error regarding requirement of any module then install them.

     In this folder there are seven files now we are going to discuss how to use each of them and in which order you have to execute each of the files.

1) get.py   : --> Used to capture and Save Images from Video for calibration 
              --> The script will open a video stream from the specified camera (index 1).
              --> Press the 's' key to save the current frame as an image.
              --> Press the 'Esc' key to exit the script.
              --> You can adjust the frame width and height by modifying the frame_width and frame_height variables in the script acoording to your requirements.
              --> Images are saved in the images/ directory with filenames in the format imgN.png, where N is an increasing number starting from 0.
              --> Make sure the images you use are of chessboard as we will use these images in the other file.Also make sure you take multple images atleat 30
                  by keeping the chessboard in different positions

2) in_ex.py : --> Used to find the Camera Calibration parameters which are intrinsic parameters and undistortion coefficients.
              --> The script reads images containing a chessboard pattern from a specified folder. Make sure to provide the correct folder path.
              --> Make sure to mention the correct dimensions of the chessboard that you have used and also the size of the chessboard square in mm.
              --> After executing this file observe the reprojection error and make sure that it is very small.

3) util.py  : --> Provides functions determine the lower and upper HSV color range limits based on given BGR color values.    
              --> Converts a given BGR color value to its corresponding HSV color space and Calculates the lower and upper
                  HSV color range limits by adjusting the hue value within a range specified accordingly.
              --> The hsv limits are calculated specifically for yellow and blue colors in the provided code.

4) test.py :  --> The script loads the camera calibration parameters (camera matrix, distortion coefficients) from saved numpy files.
              --> It undistorts the input frames using the camera calibration parameters.
              --> It performs color segmentation to detect yellow and blue objects using the provided HSV color range limits.
              --> Contours are found for detected objects, and bounding rectangles are drawn around them.
              --> Depth is calculated from camera to the plane in which we are placing the objects.
              --> The script calculates the pixel coordinates of the object's centroid and converts them to world coordinates using the known depth.

              Calculation of scaling factor: 
                 --> Place an object somewhere in the middle of the frame.
                 --> Press 'P' to print the camera coordinates and pixel coordinates of the object.
                 --> The camera coordinates are not correct and need to be scaled 
                 --> For this process displace the object in x direction first, Example by 2cm every time and note the difference in the x-coordinate
                     by printing the camera coordinates.
                 --> Now divide the 2cm that is the distance of displacement by the difference in the x-coordinate observed to obtain the scaling factor.
                 --> Do this many times and take the average of the scaling factor.
                     Similarly you can do this for y-coordinate.
                 --> Note: while displacing the object assume it as a point object located at centre of the object.
                     The z in the code is the depth of the object from camera.

              --> For calculating the coordinates of the object with respect to base we need to first make a table of pixel coordinates and it's corresponding x,y coordinates 
                  with respect to base.This can be done by placing the object at a known x,y position with respect to base then printing the pixel coordinates of the object
                  by pressing 'p'. Do this for atleast 10 different positions.
                                        
 5) uv.py  :  --> After creating the table note them in this script and then just run the script to get the extrinsic parameters of the camera and use them in the calculation 
                  of the coordinates with respect to base.

 6) pick_place_final.py : Object Detection and Arduino Integration:
                          --> This Python script performs real-time object detection using color segmentation and sends x,y,z coordinates of the object to Arduino. 
                          --> It detects yellow and blue objects in the camera's field of view, calculates their world coordinates.
                          --> The script loads the camera calibration parameters (camera matrix, distortion coefficients) from saved numpy files.
                          --> It captures frames from the camera and undistorts them using the camera calibration parameters.
                          --> Object detection is performed by segmenting yellow and blue objects using HSV color range limits.
                          --> Contours are found for detected objects, and their centroids are calculated in the camera frame.
                          --> World coordinates are calculated for the detected objects using camera calibration parameters and their pixel coordinates.
 
                          Arduino Integration for Motion Control:
                          --> The script communicates with an Arduino board via serial communication to control robotic arm.
                          --> It sends control commands to the Arduino based on the detected object's positions and constraints.

                          --> Connect an Arduino board to the computer.
                          --> Adjust the Arduino port (COM7 in this case) and baud rate (9600) in the script to match your setup.
                          --> Run the script to start real-time object detection and motion control.
                          --> Place yellow and blue objects within the camera's field of view to interact with them using the robotic arm.

                         Notes:
                         --> Adjust the thresholds and constraints for object detection and motion control based on your specific requirements and setup.
                         --> Ensure proper calibration of the camera for accurate object detection and world coordinate calculation.

7) pick_and_place.ino :  Robotic Arm Control with Arduino:
                         --> This Arduino script controls a robotic arm based on input coordinates received over a serial connection. 
                         --> It calculates the necessary joint angles to reach the specified coordinates and controls the servos accordingly to move the robotic arm 
                             to the desired position.
                         --> The script communicates over a serial connection to receive target coordinates and placement instructions.
                         --> It computes the joint angles required to position the end effector of the robotic arm at the specified (x, y, z) coordinates in 3D space.
                         --> Inverse kinematics equations are used to calculate the joint angles based on the geometric relationships of the robotic arm's links.
                         
                         Servo Control:
                         --> The calculated joint angles are converted to PWM (Pulse Width Modulation) signals, which are used to control the servo motors connected to the 
                             robotic arm's joints.
                         --> Refer to the datasheet of the servo motor for pwm information.
                         --> PWM signals with specific durations are sent to each servo motor to move the corresponding joint to the desired angle.
                         
                         Placement Handling:
                         --> The script handles instructions for placing objects by adjusting the base angle of the robotic arm accordingly.

                         Hardware Setup:
                         --> Connect the servo motors pwm pins to the appropriate pins on the Arduino board.Provide an external power supply for 5Volts and ground connections.

                         Software Setup:
                         --> Upload the provided Arduino script to the Arduino board using the Arduino IDE.
                         --> Adjust the pin assignments (base, elbow, shoulder, wrist, end_effector) in the script to match the physical connections of the servo motors.

                         --> Upon receiving coordinates and placement instructions, the Arduino script calculates the joint angles and moves the robotic arm accordingly. 
                             After completion of placing an object it sends back a message to python script saying that it is ready to receive the new coordinates of another object.
  
                         Notes:
                         --> Adjust the constants (max_limit, min_limit, a1, a2, y1, y2) in the script to match the physical dimensions and constraints of the  robotic arm. 
                             Where a1 and a2 are the joint link lengths of robotic arm.
                         --> Make sure that you run the Arduino code first in the Arduino IDE before running the main code in python.
   
   How to run:
   -----------
   1) Capture chessboard images for calibration using get.py.
   2) Run in_ex.py to find camera calibration parameters.
   3) Utilize util.py for HSV color range limit calculations:
      -> This step is not directly executed but is imported by other scripts (such as test.py) for color segmentation and object detection.
      -> Ensure util.py is placed in the same directory as the other scripts for proper import.
   4) Utilize test.py for real-time object detection, coordinate calculation and scaling factor.
   5) Update uv.py with pixel-coordinate table and execute to obtain camera extrinsic parameters.
   6) Upload pick_and_place.ino to the Arduino board using the Arduino IDE.
   7) Run pick_place_final.py to integrate object detection with Arduino for robotic arm control.

   Project Team:
   -------------
   Charitesh - 210020007
    Karthik  - 210020006
   Varshith  - 210020052
   
   Guidance and supervision provided by:
   -------------------------------------
   Prof. Ameer Mulla, Department of Electrical, Electronics, and Communication Engineering, Indian Institute of Technology Dharwad.
   
   

