import socket
import json
import numpy as np

""" a class to access optitrack data with socket connection to camera_DAQ.py"""

class OptitrackInterface:

    def __init__(self):
        

        self.pose_values_prev = np.zeros(7)
        self.time_stamp_prev = 0

        self.sock = socket.socket()
        self.port = 12345 # Use the same port number here as you did in the server script.


    def connect(self):

        self.sock.connect(('127.0.0.1', self.port)) 
        print("OptiTrack interface socket opened")


   
    def get_pose(self):

        self.sock.sendall(b'Reading')           #some kind of signal needed to receive data
        data = self.sock.recv(1024)             #wait until data is sent
        data = json.loads(data.decode())     # loads received json as dict of arrays with keys and lengths: "Time", 1;  "Position", 3; and "Orientation",4 (quaternion components) 
        return self._pose_to_np_array(data)  # returns 14 value numpy array with pose and change of pose, no time stamp 

         
    def _pose_to_np_array(self, pose_dict):
            
            """ calculates position and orientation change from previous pose and combines with the current as a 14 value numpy array"""

            time_stamp = pose_dict["Time"][0] # time in seconds
            pose_values = np.array(pose_dict["Position"] + pose_dict["Orientation"])

            dt = time_stamp - self.time_stamp_prev
            pose_values_dot = (pose_values - self.pose_values_prev) / (dt)

            self.pose_values_prev = pose_values
            self.time_stamp_prev = time_stamp

            return np.append(pose_values, pose_values_dot) # np array len 14

    def close(self):
        self.sock.close()
        print("OptiTrack interface socket closed")

        

