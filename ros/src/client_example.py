import socket
import time, json

def listen_sim():
    connected = False
    try:
        s = socket.socket()          
        port = 12345                           #Use the same port number here as you did in the server script.
        s.connect(('127.0.0.1', port)) 
        connected = True
        while True:
            time.sleep(10)  
            s.sendall(b'Reading')               #a signal to show the program is ready to receive data
            data = s.recv(1024)                 #wait until data is sent
            data = json.loads(data.decode())    #loads and decodes the binary stream
            pos = data.get("Position")          #example on how to access the json data
            orientation = data.get("Orientation")
            t = data.get("Time")
            print(data)
            
    except KeyboardInterrupt:
        if connected:
            s.close()

if __name__ == "__main__":
    listen_sim()
