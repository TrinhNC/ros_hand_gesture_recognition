# ros_hand_gesture_recognition
A ROS package for estimating hand pose using Mediapipe (and Python).

# Installation
1. Install all of the following dependencies using `pip` or `conda`:
* mediapipe 0.8.1
* OpenCV 3.4.2 or Later
* Tensorflow 2.3.0 or Later
* tf-nightly 2.5.0.dev or later (Only when creating a TFLite for an LSTM model)
* scikit-learn 0.23.2 or Later (Only if you want to display the confusion matrix)
* matplotlib 3.3.2 or Later (Only if you want to display the confusion matrix)

2. Clone this repo into a `src` folder of a catkin workspace: 
    ```
    $ git clone https://github.com/TrinhNC/ros_hand_gesture_recognition.git
    ```
  
3. Build this package by running: ```catkin build``` or ```catkin_make``` in a terminal in the worspace folder. For example: 
   ```
   $ cd ~/catkin_ws
   $ catkin build
   ```
# Run a demo
 
1. Source the workspace:
   ```
   $ source ~/catkin_ws/devel/setup.bash
   ```
2. Launch the image publisher ([here](https://github.com/TrinhNC/my_cam))
    ```
    $ roslaunch my_cam my_cam.launch
    ```
3. Launch the hand pose recognition:
   ```
   $ roslaunch ros_hand_gesture_recognition hand_sign.launch
   ```
<img src="https://user-images.githubusercontent.com/19979949/210186155-c21b0fb2-84ba-430c-94ab-b273f5f36c6c.gif" width="400" height="400" />

# Training new hand gesture
