import vrep

class Map(object):
    
    def __init__(self, x,y,z):
        vrep.simxFinish(-1) # close all opened connections
        self.clientID = None
        self.x = x
        self.y = y 
        self.z = z
        self.ref_obj = 23
        
    def start_connection(self, server, port):
        self.clientID = vrep.simxStart(server, port, True, True,5000,5) # start a connection
    
    def stop_connection(self):
        vrep.simxFinish(self.clientID) # fechando conexao com o servidor
        print('Conexao fechada!')
    
    #Return the position of some object in the map
    def get_position(self, obj, mode = vrep.simx_opmode_buffer):
        err_code, pos = vrep.simxGetObjectPosition(self.clientID,obj,-1,vrep.simx_opmode_streaming)
        return pos

class SensorVision(object):
    
    def __init__(self, clientID):
        self.clientID = clientID
        self.vision_sensor = vrep.simxGetObjectHandle(self.clientID,"vs", vrep.simx_opmode_blocking)[1]
        self.resolution  = None
        self.image = None
        
    def init_sensor(self):
        vrep.simxGetVisionSensorImage(self.clientID,self.vision_sensor, 0, vrep.simx_opmode_streaming)
    
    @property
    def get_image(self):
        err_code, res, image = vrep.simxGetVisionSensorImage(
            self.clientID, self.vision_sensor, 0, vrep.simx_opmode_buffer
        )
        
        return image