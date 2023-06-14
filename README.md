# Co-adaptation with RealAnt

This repository contains the sourcecode for our AEE project: "Creating a Robotic Surrogate for Co-evolving Behaviour and Design in the Real World". In the project, we developed a robot platform using the RealAnt robot and a OptiTrack motion capture setup. Building the platform involved modifying a data-efficient reinforcement learning-based co-adaptation algorithm (from https://github.com/ksluck/Coadaptation) for the needs of the real world and developing new software for reading the robots pose, generating parts from design parameters and controlling the robot.  

## Contents

`STL_Generation` - STL file generation for 3D printing optimized leg segments

`ant11_cmd_dxl_mod` - A control program for the RealAnt's OpenCM9.04 board, modified from the original RealAnt project to also return speed and load of the servos. 

`project14-COADAPT` - The modified co-adaptation program, includes new `realant_environment` package for controlling the RealAnt robot

`ros` - Files for ROS Melodic workspace to read data from the Optitrack setup

More information of the platform's components including instructions for installion are found in their respective folders. Instructions to run the Co-adaptation program with RealAnt is found in the `realant_environment`.


## Links

- A guide to setup the ARDUINO IDE developing environment for the OpenCM9.04 board's control software: 
https://emanual.robotis.com/docs/en/parts/controller/opencm904/#development-environment
- the github repository for the co-adaptiation algorithm: https://github.com/ksluck/Coadaptation
- the github repository for the RealAnt: https://github.com/OteRobotics/realant#realant
