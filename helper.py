import numpy as np
import vrep

class Simulation(object):
    
    def __init__(self):
        vrep.simxFinish(-1) # just in case, close all opened connections
        self.clientID = None
        self.x_limit = [-9, 9]
        self.y_limit = [-9, 9] 
        self.z_limit = [0, 2]
        self.start = True
        self.v_start = 0.2
        self.x = -0
        self.y = -9
        self.z = 2
        self.START = True
        self.SEARCH = False
        self.vx_search = 0.01
        self.x_enable = True
        self.vy_search = 0.01
        self.y_enable = False
        self.y_control = 0
        self.boss = False
        
    def start_connection(self, server, port):
        self.clientID = vrep.simxStart(server, port, True, True,5000,5) # start a connection
    
    def stop_connection(self):
        vrep.simxFinish(self.clientID) # fechando conexao com o servidor
        print('Conexao fechada!')
        
    def get_position(self, obj, mode = vrep.simx_opmode_buffer):
        err_code, pos = vrep.simxGetObjectPosition(self.clientID,obj,-1,vrep.simx_opmode_streaming)
        return pos