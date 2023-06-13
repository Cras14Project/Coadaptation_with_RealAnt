# Camera module
This README desribes the camera module created during a school project for the use of obtaining and using data from an Optitrack system (https://optitrack.com/). 

The module consists of several seperate pieces, mainly Optitrack's Motive software that is used to stream the data into LAN network (https://optitrack.com/software/motive/), a rigid body ROS publisher downloaded from https://github.com/ros-drivers/mocap_optitrack/tree/foxy-devel and a script written by us to subscribe to this data and then share it to our own RL program. 

It is good to note that the subscriber provided in this repository can also be used in other applications by e.g., modifying the name and type of the subscribed topic and the callback-function. 
## Libraries & Environments
The OS used for this project was Ubuntu 18.04. Due to use of ROS it is recommended to download a virtual machine when working on any other OS than Linux.

The script camera_DAQ.py works on ROS Melodic and utilizes rospy. It might also work on other ROS environments, but not on ROS2. 
A link on where to install ROS, ubuntu 18.04 recommended: http://wiki.ros.org/melodic/Installation

The code was written with Python 2, and uses the inbuilt libraries such as socket, and json.

## How To Use
After installing all the necessary packages and setting up the ROS environemnt (refer to the ROS-part of this document):
1. Navigate to the ROS environement folder
2. Run a publisher (e.g. mocap_optitrack or the example coming with our repository)
3. Run the camera_DAQ.py file with "rosrun [package name] camera_DAQ.py"
4. Run a seperate client program (Co-Adaptation module, or our example file)

## OptiTrack
Here are some links to get started with new Optitrack setup and how to use it with ROS (concerning our project):
http://wiki.ros.org/mocap_optitrack
https://docs.optitrack.com/motive/data-streaming
https://tuw-cpsg.github.io/tutorials/optitrack-and-ros/

To summarize:
1. Setup the Optitrack cameras
2. Calibrate and mask reflections
3. Create rigid body for the object you will be tracking
4. Select the created body from x-pane and start the streaming from the streaming-pane (make sure the options are selected correctly, e.g. the rigid body streaming number, and the ip-address)
5. The system should be ready for data acquistion.

## ROS
The scripts mentioned in this document are contained in a ROS package. This means that the scripts can be run within the ROS environment, with the command "rosrun [package_name] [file_name]". Always remember to run **"catkin_make"** before launching a ROS application for the first time, and after making any changes to the code. 

**Linux/Ubuntu:** When downloading Camera module's files, remember to give them the execution right to run them through the terminal with the command: 
**"chmod +x [file_name]"**

If the files are run seperately, roscore needs to be launched (Testing section). If mocap_optitrack is used, roscore is launched by the roslaunch-command (under subscetion ROS publisher).

More tutorials on basic ROS usage can be found here:
http://wiki.ros.org/ROS/Tutorials

### ROS publisher
http://wiki.ros.org/mocap_optitrack
https://github.com/ros-drivers/mocap_optitrack/tree/foxy-devel

The publisher captures the data streamed by Motive and publishes it as a rostopic every 6 ms. The topic is roscpp/rospy PoseStamped datatype.

The publisher can be run with the command: **"roslaunch mocap_optitrack mocap.launch"**

## Main Script (camera_DAQ.py)
The main script written by us is called "camera_DAQ.py". It contains a simple Subscriber-class and a main-function that initiates the process. 

The Subsriber class consists of three functions:

init: 
    Input: self, Socket
    Output: None

callback:
    Input: self, Data (PoseStamped/String)
    Output: None
    Purpose: A function called by the subscriber whenever new data is detected. 

convert_to_binary:
    Input: Self, Data to be converted (PoseStamped/String)
    Output: Binary version of the input
    Purpose: Conversion from String/PoseStamped to binary/json

The main-function creates a ROS node and the socket that acts as a server, then waits for a connection from other program that wants to access the data. 

## Testing
The camera_DAQ script can be tested by using our example python files (in this folder) in the following way:
(Note! It is expected you've already set up the ROS environement and are able to execute the following files)
1. Navigate to the ROS environment folder
2. Run roscore with the command **"roscore"**
3. Run the example publisher with the command **"rosrun ros publisher.py"**
4. Run the server code with **"rosrun ros camera_DAQ.py"**
5. Run the client example from an IDE or the terminal with "python -m client_example.py"


## Future Improvements
- Safe closing
- Updating the script to python 3 and the ROS environment to something that supports it, such as ROS2
--> another option is to wait to see if later ROS versions would be available as ROS2 is less well documented compared to ROS.
