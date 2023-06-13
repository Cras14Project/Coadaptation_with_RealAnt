## robot.py

The Robot class communicates with the RealAnt robot via socket connection. Measurements from the servos are collected in a thread that updates the 24 values to a shared deque. The thread is modified from the same function in`ant_server.py` of [the original RealAnt project](https://github.com/OteRobotics/realant).  The main methods of the class are `apply_action` and `calc_state`.

The robot class translates the algorithms actions as relative position changes in the servos range and sends them using the connection in the `apply_action` method. The angular distance the servos can move in a single action is limited to half of the full motion range of the servo, but if the command exceeds the bounds given inside the robot class, the servos new reference position is given at the bound. The sending of the calculated action is split in two: a command is first sent to the robot’s horizontal joints and a small time later to the vertical joints. This is a fix to prevent overloading the servo command which happens every once in a while when sending a full command. The overloading results in the servos detaching torque the middle of a training session.

The method`calc_state` returns the state and the reward of the robot environment. It combines internal state (servo positions, velocities, and loads) and the external state (robot position and orientation cartesian and quaternions and their rate of change ) as single state. The internal state is accessed from a deque that is updated by a thread. The load and velocities are converted to a linear range ([see dynamixel AX-12A documentation on present speed and load](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#present-speed)). The external state is forwarded from the Optitrack using the `OptitrackInterface` class. The class opens a socket connection to the robot’s pose topic subscriber of `camera_DAQ.py`.

## robot_env.py

`RobotEnv` is a `Gym Env` class, which interfaces with the coadaptation program. The basic gym functions are overwritten with the use of the robot classes methods. The design optimization aspect is implemented the same way as in the `HalfCheetahEnv` class. However, the amount of design parameters are changed. The class considers eight design parameters: length and tip radius for each leg. The class also implements the pausing as well as saving and loading the evo replay parameters. The loading and pausing methods are discussed in the PROJECT14-COADAPT folders readme.

## optitrack_interface.py

This file houses the `OptitrackInterface` class, which is used to connect to `camera_DAQ.py` files ROS subscriber. the ROS subscriber sends the robot's pose as a Json-file via socket. In the `get_pose` method (used in `calc_state`), the Json-file is received and loaded as a dictionary and having saved the previous pose, the class calculates the pose change and combines it with the pose. The method returns the external state as a NumPy array.

## Before running the coadaptation program with RobotEnv

Make sure to do the following before running:

1.  Install required dependencies.

2.  Place the absolute path of the `robot_environment` folder to the `coadapt.py` file's `sys.path.append()` on line 2.

3.  After plugging the robot, find out what USB port is used. the correct port can be found by writing `ls /dev/serial/by-id -l` in terminal. The correct port should be given to `usb-ROBOTIS_ROBOTIS_COMPORT-IF00` as `../../[port name]`. Place the port in the robot.py file in line 227 as an argument to the `serial.Serial` object. The found name is written as `“dev/[port name]”`.

4. Export rlkit python path with the command: `export PYTHONPATH=[absolute path of rlkit folder]`

## How to run

Running the coadapt program with RealAnt and OptiTrack:

1. In the ROS workspace run the following:
 ```
Roscore
Roslaunch mocap_optitrack mocap.launch
Rosrun pw camera_DAQ.py
```

2. Run the coadaption program in the `coadaptation` directory with: `python main.py sac_pso_batch`

## Notes and possible errors

- Make sure to run `camera_DAQ.py` before the main program. Otherwise the program will get stuck after the prompt "Optitrack interface socket opened". 

- After launching the program again with the robot, it will sometimes throw Index error for the deque. This  means the robot is not sending data to the serial connection. If this happens the robot must be reset. Simply re-plugging the USB connection after a short wait should work.

## Future improvements

- A choice was made to connect directly to the robot in the `Robot` class with a serial connection instead of connecting with a socket connection to the `ant_server.py` and `antproxy.py`  which the original project used to connect with the serial link. As second thought, I believe the reset error might have been avoided by keeping the connection to the robot intact in a separate program, such as using the `ant_server.py`.

- Right now, the joint limits are set so that the robots’ legs don’t collide with certain leg parameters and the class doesn’t consider updating these. Some method in robot.py could be used the joint limits according to the robot’s updated design parameters. However, determining the limits would be complex when each leg has its own design, and it might be difficult to the policy to adapt to the changing limits with new legs.
