# ROS Hand Gesture Recognition
A ROS package for estimating hand pose using Mediapipe (and Python). For more details, please check out my blog post [here](https://robodev.blog/hand-gesture-recognition-in-ros).

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

# Train new hand gesture
The current package can classify only six signs (classes) and I labeled them: Go, Stop, Forward, Backward, Turn Right and Turn Left (see the image below). I named them like that because they will be converted to control signals to move a robot later in this series. If you want to change or add gestures, or you find out that my trained model does not perform very well with your hand, you can collect data and train it again by yourself.

![image](https://user-images.githubusercontent.com/19979949/210246206-dac0839b-d079-4d7e-8b23-89bc9bdc2294.png)

There are two jupyter notebooks included in the folder src/notebooks:

* [keypoint_classification_EN.ipynb](https://github.com/TrinhNC/ros_hand_gesture_recognition/blob/main/src/notebooks/keypoint_classification_EN.ipynb): a model training script for hand sign recognition.

* [point_history_classification.ipynb](https://github.com/TrinhNC/ros_hand_gesture_recognition/blob/main/src/notebooks/point_history_classification.ipynb): a model training script for finger gesture recognition (meaning the model can detect the movement of your fingers and not just a static sign like in the keypoint classification).

I used only the keypoint classification model in the current ROS package because it is enough for the application but you can feel free to adjust it to match yours.

In the example below, I will show you how to add one more sign to the detection. Let's say we want to add this sign✌️and name it "Hi".

![image](https://user-images.githubusercontent.com/19979949/210246331-a985c2e4-5a99-4bf5-9020-55b967d5b87a.png)

First, open the keypoint_classifier_label.csv in the folder src/model/keypoint_classifier. Here you find all the labels (at the moment 6 classes) and you should add 'Hi' to the end.
Next, you need to record data and append it to the file keypoint.csv in the folder src/model/keypoint_classifier. If you open this file, you will see it contains 6410 lines. The first number in each line is the class ID with respect to the list above, for example, "Go" has ID 0, "Stop" has ID 1, and so on. Then comes 42 numbers or 21 pairs of numbers which represent the coordinates of each keypoint (i.e. the hand knuckle) with respect to the origin which is the wrist. One thing to note is that the IDs in the image below are the key point IDs and they are different from the class IDs.

![image](https://user-images.githubusercontent.com/19979949/210246469-235c31e8-879a-4089-a6d1-3fbe75223926.png)

In order to record data, open the script app.py:
```
python3 app.py
```
Then press k on the keyboard. You should see the line MODE: Logging Key Point shows up. Then, use your right hand to make the target sign✌️visible and press and hold the number 6 (class ID of "Hi") with your left hand. This will append the new data to the file keypoint.csv until you release the key. You can also try to press & release 6 immediately and check the file. It should have one new line at the end starting with the number 6 and a list of numbers that follow. Also, during the recording, remember to move your right hand to different positions to make the dataset varied.

<img src="https://user-images.githubusercontent.com/19979949/210239140-cd5998ca-5937-48f4-9f91-8a86ca10da40.gif" width="400" height="400" />

After recording for about 10-15 seconds, the data should be ready and you can stop the program. Open the notebook file [keypoint_classification_EN.ipynb](https://github.com/TrinhNC/ros_hand_gesture_recognition/blob/main/src/notebooks/keypoint_classification_EN.ipynb). Edit `dataset`, `model_save_path` and `tflite_save_path` to match your paths. Change NUM_OF_CLASSES to 7 instead of 6: `NUM_CLASSES = 7`. Then run the notebook from beginning to the end. The training is executed in cell [13] and takes around 2-3 minutes. After that you can launch `my_cam.launch` and `hand_sign.launch` to see the result like below.

<img src="https://user-images.githubusercontent.com/19979949/210258993-de24fcb4-4e1c-44f6-b1c0-6088677c0691.gif" width="400" height="400" />
