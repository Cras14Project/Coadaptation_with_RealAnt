import time
import numpy as np
import serial
import threading
from optitrack_interface import  OptitrackInterface
from collections import deque


# a global variable to stop read_measurements loop to close thread after finishing main
stop = None

def read_measurements(ser, deque):

    """ reads the robots measurements through serial connection and converts to int array and adds them to a shared deque """

    valcnt = 0
    global stop
    last_ant_time = 0
    while not stop:
        try: 
            line = ser.readline().decode()
        except UnicodeDecodeError:
            line = ser.readline().decode()

        if line.startswith("- recv"):
            #print(line.strip("\r\n"))
            pass

        elif line.startswith("meas "):
            
            vals = line[5:].split("\t")
            vals = vals[:-1] # read measurements: time stamp, 8 servo positions, 8 servo speeds, 8 servo loads 

            if len(vals) == 25:

                vals = list(map(int,vals)) #convert from string to int

                if valcnt % 50 == 0:
                    #print(line.strip("\r\n"))
                    pass
                valcnt += 1 

                ant_time = vals[0]

                if ant_time - last_ant_time < 0:
                    print("implausible ant time, skipping", vals)
                    continue

                last_ant_time = ant_time

                deque.append(vals[1:]) #put to the shared deque, replaces the one and only deque value, removes time stamp

        else:
            if line.strip("\r\n ") != "":
                #print(line.strip("\r\n"))
                pass


class Robot:

    """ a class to communicate with the RealAnt robot"""

    def __init__(self):

        horizontal_limits = (307, 717) # Horizontal limits preventing collision of white legs (~6.5 cm)
        vertical_limits = (307, 717)   # This could be higher
        self.jointLimits = [horizontal_limits, vertical_limits] * 4  # only affects this class so that commands are given in the right range, servo control table max and min should be at the default (0-1023)

        self.internal_state_array = None                 #np array
        self.external_state_array = None                 #np array

        self.ser = None                                  # for robot communication

        self.ant_obs_thread = None                       # for opening/closing the thread
        self.deque = deque(maxlen=1)                     # for safe read/write between main thread and ant_obshread

        self.camera_data_handler = OptitrackInterface()  #the class to connect to camera_DAQ.py for optitrack data
        
    # CALCULATING STATE

    def calc_state(self):

        """ combines measurements from the robots joints and the optitrack as a state and calculates a reward"""

        #TODO: scale ant observations smaller
        
        try:
            ant_state_array = self.deque[-1] #get all 24 servo values
        except IndexError:
            raise IndexError(" no values received from robot - need to reset robots OpenCM9.04 board")
            

        self.internal_state_array = np.array(self._values_to_linear_range(ant_state_array)) #modifiy values of load and velocity to linear range 

        self.external_state_array = self.camera_data_handler.get_pose()
        
        state = np.append(self.internal_state_array, self.external_state_array)

        reward = self.reward()

        return state, reward
    
    def _values_to_linear_range(self, internal_state):
        
        """returns the full state list, with velocity and load readings transformed from range (0,2047) to (-1023, 1023), with postive direction as CCW. 
        position kept in tact, in the range of the set joint limits
        """

        return internal_state[0:8] + [-1 * (x - 1024) if 1024 <= x <= 2047 else x for x in internal_state[8:]]
    
    def reward(self):


        # speed reward
        x_speed = self.external_state_array[7] 

        # try to lift robot from ground
        y_height = self.external_state_array[1] # y component of optitrack pose 

        # reward based on servo positions, the further from the limit, the higher the reward  //  probably useless with collision avoiding servo limits

        servo_pos_distance_to_limit_sum = 0
        for i in range(len(self.jointLimits)):
            min_pos, max_pos = self.jointLimits[i]
            servo_pos  = self.internal_state_array[i]
            if servo_pos < 512:
                distance_to_limit = servo_pos - min_pos
            else:
                distance_to_limit = max_pos - servo_pos
            servo_pos_distance_to_limit_sum += (distance_to_limit / 512)**2
        servo_pos_distance_to_limit_sum / 8

        # set the scale accordingly

        k1 = 1        
        k2 = 1

        return x_speed  + k1 * servo_pos_distance_to_limit_sum + k2 * y_height

    # APPLYING ACTIONS FROM ALGORITHM

    def apply_action(self, action):

        """ Sets all joints in the correct range [min max] and sends a command to the pcb"""
        
        assert len(action) == 8

        servo_command_strings = ['']*8
        current_positions = self._get_servo_positions()
        
        # fills a list with commands in string

        for i in range(8):
            reference_position = self._alg_value_to_servo_reference_pos(action[i], current_positions[i], self.jointLimits[i])
            servo_command_strings[i] = f"s{i+1} {reference_position:.0f}"
        
        cmd_string_horizontal = " ".join(servo_command_strings[1::2])+"\n"   #splitting command to vertical and horizontal joints
        cmd_string_vertical = " ".join(servo_command_strings[::2])+"\n" 

        #print(bytes(cmd_string_horizontal, 'utf-8'))
        #print(bytes(cmd_string_vertical, 'utf-8'))
       
        self.ser.write(bytes(cmd_string_horizontal, 'utf-8'))  #send the commands seperately to avoid command overload (servo leds blink red and torque is lost for few seconds)
        time.sleep(0.05) # tune this 
        self.ser.write(bytes(cmd_string_vertical, 'utf-8'))

    def _get_servo_positions(self):

        "reads the deque for latest servo positions for apply action"

        return self.deque[-1][:8]

    def _alg_value_to_servo_reference_pos(self, alg_value, pos_prev, joint_range):
        
        """ A function to transform a single joint value from algorithms range (-1, 1) to robots range, as relative movement.
          With a single action the joint can move as much as HALF of the servos FULL RANGE 
          example: alg_cmd = 0.5 pos_prev = 100 -> new_servo_goal_pos = 0.5 * 512 + 100 = 356
          Postive direction: CCW 
        """
        
        min_pos, max_pos = joint_range
        servo_reference_pos = alg_value * 512 + pos_prev
        servo_reference_pos = int(round(servo_reference_pos))

        if servo_reference_pos < min_pos:
            servo_reference_pos = min_pos
        if servo_reference_pos > max_pos:
            servo_reference_pos = max_pos

        return servo_reference_pos

    # NAMED COMMANDS

    def reset(self):
        
        """ sets servos to middle - 512 """
        
        self.ser.write(b"reset\n")
        time.sleep(1.0)

    def attach_servos(self):

        """enables torque"""

        self.ser.write(b"attach_servos\n") 
        time.sleep(1.0)

    def detach_servos(self):
        
        """disables torque"""

        self.ser.write(b"detach_servos\n") 
        time.sleep(1.0)

    # OPEN/CLOSE CONNECTIONS 

    def open_all_connections(self):
        
        self.open_serial_connection()
        self.start_ant_obs_thread()
        self.camera_data_handler.connect() # start socket connection to ROS side camera_DAQ.py // remember to start camera_DAQ.py before coadapt
        self.attach_servos()


    def open_serial_connection(self):

        """ opens serial port,  to find port with linux: write "ls /dev/serial/by-id -l" in terminal, The port should match usb-ROBOTIS_ROBOTIS_ComPort-if00  """

        self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)

    def close_serial_connection(self):

        self.ser.close()

    def start_ant_obs_thread(self):

        global stop
        stop = False
        self.ant_obs_thread = threading.Thread(target=read_measurements, args=(self.ser, self.deque))
        self.ant_obs_thread.daemon = True 
        self.ant_obs_thread.start()
        time.sleep(2)

    def close_ant_obs_thread(self):

        global stop
        stop = True
        self.ant_obs_thread.join()

    def close_all_connections(self):

        self.detach_servos()
        self.close_ant_obs_thread()
        self.close_serial_connection()
        print("Robot socket closed")    
        self.camera_data_handler.close()
        print("closing robot complete")
    
    # PRINT

    def print_state(self):

        """ prints the latest calculated state """

        print("observations from robot:")

        print("\tservo positions:", self.internal_state_array[:8])
        print("\tservo velocities:", self.internal_state_array[8:16])
        print("\tservo loads:", self.internal_state_array[16:])
        
        print("observations from optitrack")

        print("\tposition: x = {}, y = {}, z = {}".format(*self.external_state_array[:3]))
        print("\tquaternion components: x= {}, y = {}, z = {}, w = {}".format(*self.external_state_array[3:7]))
        print("\tspeed: x = {}, y = {}, z = {}".format(*self.external_state_array[7:10]))
        print("\tquaternion component change: x = {}, y = {}, z = {}, w = {}".format(*self.external_state_array[10:14]))