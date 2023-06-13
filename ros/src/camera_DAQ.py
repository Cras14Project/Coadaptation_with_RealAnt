#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import socket, json


#Simple subscriber class using rospy. Can be used to get data from a publisher.
class Subscriber():
    
    def __init__(self, socket):
        #self.sub = rospy.Subscriber("/pose", String, self.callback) # FOR TESTING SWITCH
        self.sub = rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, self.callback)     #creates a rospy Subscriber of type PoseStamped for later use
        self.socket = socket                                                                    #saves the found socket for sending data

    def callback(self, data):
        binary_data = self.convert_to_binary(data)                                              #converts the data to binary
        #binary_data = self.convert_to_binary(PoseStamped()) # FOR TESTING SWITCH

        try:
            flag = self.socket.recv(20, socket.MSG_DONTWAIT | socket.MSG_PEEK)                  #check if data is asked to be sent without blocking
            flag = self.socket.recv(20)                                                         
            self.socket.sendall(binary_data)                                                    #sends the data as a whole (cmp. to "socket.send")
            
        except:
            pass
    
    #a function to seperate PoseStamped into json format, to be able to send it through the binary stream
    def convert_to_binary(self,data):
        if type(data) == String:
            return (str(data)).encode("ascii")
        elif type(data) == PoseStamped:
            time = [data.header.stamp.secs + data.header.stamp.nsecs * 10**(-9)]
            arr1 = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
            arr2 = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
            return json.dumps({"Time": time, "Position": arr1, "Orientation": arr2}).encode()
        else:
            return b"moi"
            

#function for subscribing to the data. Calls subscriber with rospy.spin()
def main():
    connected = False
    try:
        rospy.init_node('listener', anonymous=True) 
        s = socket.socket() 
        port = 12345                                #make this any random port
        s.bind(('127.0.0.1', port))                 #localhost loopback
        s.listen(1)                                 #put the socket into listen mode
        c, addr = s.accept()                        #wait for 1 connection befor executing rest of the code, blocks the execution until connection achieved
        connected = True
        sub = Subscriber(c)
        rospy.spin()                                #rospy.spin() blocks until the script is terminated
    
    except Exception as e:                          
        print(e)
        if connected:
            print("Closing the socket.")
            c.close() 

        print("Optitrack subscriber closed.")



if __name__ == '__main__':
    main()        
