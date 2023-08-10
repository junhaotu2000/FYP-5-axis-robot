# FYP-5-axis-robot
These are the files developed in Yihan LIU's Final Year Project, a 5-axis vision articulated robot
The project is developed into 4 sections
## (1) Mechanical Structure
This includes the CAD files of designed robot, whcih is designed base on BCN3D Moveo.  
CAD file is designed in the SOLIDWORKS.  
The information about BCN3D Moveo could be found in https://github.com/BCN3D/BCN3D-Moveo.  
Joint 1 of Moveo has been redesigned to use a planetary reducer replacing the timing belt, which causes the joint 1 could rotate 360 degrees  
Diagram of planetary reducer could be demonstrated as  
<img src ="https://github.com/YihanLIU0224/FYP-5-axis-robot/assets/132924198/d42aa4f6-e5d8-4da8-bdab-25102519bdbd" width = "600px">

Also, there is a fixator designed on the end effector to install depth camera. 
The disgram of designed robot could be shown as  
<img src = "https://github.com/YihanLIU0224/FYP-5-axis-robot/assets/132924198/53aaa8e7-c8a8-44ce-96c8-1dad855e63fb" width = "200px">   
Also, there are some fixators designed to install on each joint to test accuracy of rotational angle of joints, which could be shown as  
<img src = "https://github.com/YihanLIU0224/FYP-5-axis-robot/assets/132924198/a44b9272-2aa9-4ccb-aa07-1fca570815df" width = "400px">  
The processor used in this project is Raspberry Pi Model 4B.  
The camera used for vision development is Intel RealSense D435.
## (2) Robotic Kinematics  
These sections includes the MATLAB simulation codes and kinematic codes used in the Raspberry Pi.  
### (2.1) Kinematic Simulation in MATLAB (MATLAB code)
#### Code 1: forward_kinematic_matrix_calculation.m
Run forward kinematics (joint angles -> coordinates of end effector) using matrix.  
#### Code 2: forward_kinematic_equation_calculation.m
Run forward kinematics (joint angles -> coordinates of end effector) using equations.  
#### Code 3: inverse_kinematic.m
Run inverse kinematics (coordinates of end effector -> joint angles).   
#### Code 4: workspace_simulation.m
This simulation needs to install the robotic toolbox from Peter Corke.  
Details for installation of this could be found: https://petercorke.com/toolboxes/robotics-toolbox/  
Simulate the workspace of designed robot using Monte Carlo method.   
### ----------------------------------------------------------------------------------------------
### (2.2) Kinematic Code of Raspberry Pi (Python Code)  
These following codes require to install the GPIO and Python library in the Raspberry Pi OS.  
#### Code 1: end_effector.py
Code to switch ON/OFF end effector
#### Code 2: forward_kinematic.py
Code to calcualte forward kinematic equations
#### Code 3: inverse_kinematic.py
Code to calculate inverse kinematic equations
#### Code 4: joint_motion.py
Code to control the rotation of joints
## (3) Computer Vision
The following codes for computer vision require to build the environment for RealSense Camera.    
Detailed procedure for this could be seen in https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435  
Also, OpenCV and its controbution libraries should be installed to processing images.
These could be achieved by    
##### pip install opencv-python  
##### pip install opencv-contrib-python   
### ----------------------------------------------------------------------------------------------
### (3.1) Colour Recognition
#### HSV detector.py
Program to detect the values of HSV parameters of a pixel colour
#### colour_detect.py
Program to detect and measure the coordinate of object with a specific colour
### ----------------------------------------------------------------------------------------------
### (3.2) Facial Recognition
The procedures to use facial recognition code are
1. Recognize the face and save specific facial images into a database
2. Train the camera using database to generate trainer.yml
3. Recognize the specific face using trainer.yml
#### haarcascade_frontalface_default.xml
This is the classfication file used for recognizing face, which is copied from https://github.com/opencv/opencv/blob/4.x/data/haarcascades/haarcascade_frontalface_default.xml
#### data_setup.py
Code to capture the facial images for training, images will be generated in the database folder
#### trainer.py
Code to train the camera using facial images in database, output classification file is saved in trainer folder
#### facial_recognition.py
Code to recognize a specific face
## (4) Final Application
Following codes is for the applications of integrated robot. The demonstration of it could be viewed via https://uniofnottm-my.sharepoint.com/:f:/g/personal/ssyyl20_nottingham_ac_uk/Er96vf4qYWJBqkzvtPdmCy0BttAT07Y9VWQ2YcLCw33cCg?e=NZlaZr   
### (4.1) Pick and Place  
Program to pick and place different colour objects into corresponding platforms. The program has been divided into different fucntions and the control code is in the control_panel.py.
### (4.2) Search and Classification  
Program to search and classify different colour objects around the robot into corresponding platforms. The program has been divided into different fucntions and the control code is in the control_panel.py.
### (4.3) Unlocking  
Program to unlock and bow to specific face.
