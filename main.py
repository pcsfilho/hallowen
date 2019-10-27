import vrep
import time

from helper import Map
from quadcopeter import Quadcopter

if __name__ == "__main__":

    x = [-9, 9] # x map limit
    y = [-9, 9] # y map limit
    z = [0.05, 1.8] # z map limit
    server = '127.0.0.1'
    port = 19999
    
    sim = Map(x,y,z) # init map parameters
    sim.start_connection(server, port) #start connection with vrep simulation

    if sim.clientID != -1:
        print ("Connected to remote API server")
        quad = Quadcopter(sim) #init quadcopter
        quad.create_proximity_sensors() #initializing quadcopter proximity sensors
        quad.create_vision_sensor() #initializing quadcopter proximity vision sensor

        sim.get_position(quad.target, vrep.simx_opmode_streaming) # update targe position in map
        
        time.sleep(0.2)    
        #put quadcopter on initial position
        quad.initial_position()
        # #starting search path
        quad.make_trajectory()
            
        if quad.was_found:
            print('Object found')
        else:
            print('Object not found') 
    else:
        print("Not connected to remote simulation")

    vrep.simxStopSimulation(sim.clientID, vrep.simx_opmode_oneshot)