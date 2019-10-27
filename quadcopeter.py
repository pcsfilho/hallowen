import vrep
import time
from helper import SensorVision

'''
This class is responsible for controlling the quadricopter instantiated in the v-rep simulation,
which aims to find a black cube on a map.
'''
class Quadcopter(object):
    
    def __init__(self, world_map):
        self.map = world_map
        self.target = vrep.simxGetObjectHandle(self.map.clientID,'Quadricopter_target', vrep.simx_opmode_blocking)[1]
        self.fix = False
        self.sensor_vision = None
        self.was_found = False
        self.shift = 0.11
        self.foward_enable = False
        self.back_enable = False
        self.shift_enable = False
        
        self.proximity_sensor_1 = None
        self.proximity_sensor_2 = None
        self.proximity_sensor_3 = None
        self.proximity_sensor_4 = None
        self.proximity_sensor_5 = None
        

    def create_proximity_sensors(self):
        
        err_code, self.proximity_sensor_1 = vrep.simxGetObjectHandle(self.map.clientID,"s1", vrep.simx_opmode_blocking)
        err_code, self.proximity_sensor_2 = vrep.simxGetObjectHandle(self.map.clientID,"s2", vrep.simx_opmode_blocking)
        err_code, self.proximity_sensor_3 = vrep.simxGetObjectHandle(self.map.clientID,"s3", vrep.simx_opmode_blocking)
        err_code, self.proximity_sensor_4 = vrep.simxGetObjectHandle(self.map.clientID,"s4", vrep.simx_opmode_blocking)
        err_code, self.proximity_sensor_5 = vrep.simxGetObjectHandle(self.map.clientID,"s5", vrep.simx_opmode_blocking)

        vrep.simxReadProximitySensor(self.map.clientID, self.proximity_sensor_1, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.map.clientID, self.proximity_sensor_2, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.map.clientID, self.proximity_sensor_3, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.map.clientID, self.proximity_sensor_4, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.map.clientID, self.proximity_sensor_5, vrep.simx_opmode_streaming)
    
    def create_vision_sensor(self):
        self.sensor_vision = SensorVision(self.map.clientID)
        time.sleep(0.01)
        self.sensor_vision.init_sensor()
        
    
    def update_sensor(self, sensor):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.map.clientID, sensor, vrep.simx_opmode_buffer
        )
        
        return state
    
    '''
    Test if reference color object inside image
    '''
    def object_was_found(self):
        return self.map.ref_obj in self.sensor_vision.get_image
    
    '''
    Testing if there is any collision and start fix
    '''
    def test_trajectory(self):
        time.sleep(0.02)
        if self.get_proximity_sensor_4 and self.target_position[1] + 1.5 < self.map.y[1] and self.current=='L':
            self.fix_trajectory(sensor = self.proximity_sensor_4, function =  self.set_left)
        
        elif self.get_proximity_sensor_3 and self.target_position[1] - 1.5 > self.map.y[0] and self.current=='R':
            self.fix_trajectory(sensor = self.proximity_sensor_3, function = self.set_rigth)
        
        elif self.get_proximity_sensor_1 and self.target_position[0] + 1.5 < self.map.x[1] and self.current=='F':
            self.fix_trajectory(sensor = self.proximity_sensor_1, function =  self.set_foward)
        
        elif self.get_proximity_sensor_2 and self.target_position[0] - 1.5 > self.map.x[0] and self.current=='B':
            self.fix_trajectory(sensor = self.proximity_sensor_2, function = self.set_back)
        
        if  (self.target_position[2] > self.map.z[1] and not self.fix):
            self.fix_height()

    '''
    Put target quadcopter in minimum z coordinate
    '''
    def fix_height(self):
        while self.target_position[2] >= self.map.z[1] and not self.get_proximity_sensor_5:
            self.set_down()
    
    '''
    Forward movement with the proximity_sensor_1 as reference
    '''
    def set_foward(self):
        err_code = vrep.simxSetObjectPosition(
            self.map.clientID,self.target, -1,
            [self.target_position[0]+self.shift, self.target_position[1], self.target_position[2]],
            vrep.simx_opmode_oneshot
        )
        self.test_trajectory()
    
    '''
    Upward movement
    '''    
    def set_up(self):
        err_code = vrep.simxSetObjectPosition(
            self.map.clientID,self.target, -1,
            [self.target_position[0], self.target_position[1], self.target_position[2]+self.shift],
            vrep.simx_opmode_oneshot
        )
        
    '''
    Downward movement
    '''
    def set_down(self):
        err_code = vrep.simxSetObjectPosition(
            self.map.clientID,self.target, -1,
            [self.target_position[0], self.target_position[1], self.target_position[2]-self.shift],
            vrep.simx_opmode_oneshot
        )
        
    '''
    Backward movement
    '''    
    def set_back(self):
        err_code = vrep.simxSetObjectPosition(
            self.map.clientID,self.target, -1,
            [self.target_position[0]-self.shift, self.target_position[1], self.target_position[2]],
            vrep.simx_opmode_oneshot
        )
        self.test_trajectory()
    
    '''
    Right move
    '''
    def set_rigth(self):
        err_code = vrep.simxSetObjectPosition(
            self.map.clientID, self.target, -1,
            [self.target_position[0], self.target_position[1]-self.shift, self.target_position[2]],
            vrep.simx_opmode_oneshot
        )
        self.test_trajectory()
    
    '''
    Left move
    '''
    def set_left(self):
        err_code = vrep.simxSetObjectPosition(
            self.map.clientID, self.target, -1,
            [self.target_position[0], self.target_position[1]+self.shift, self.target_position[2]],
            vrep.simx_opmode_oneshot
        )
        self.test_trajectory()
     
    '''
    Fix trajectory if collision
    '''
    def fix_trajectory(self, sensor, function):
        self.fix = True
        while 1: # up while sensor is True
            if not self.update_sensor(sensor):
                count = 0
                time.sleep(0.05)
                while count <= 10 or self.get_proximity_sensor_5: # move to the function movement while proximity sensor is True 
                    function()
                    count = count + 1
                    time.sleep(0.05)
                    
                self.fix_height()
                break
            
            self.set_up()
            
        self.fix = False
    
    def initial_position(self):
        while self.target_position[2] <= self.map.z[1]+1 and not self.was_found:
            self.set_up()
            time.sleep(0.02)
            self.was_found = self.object_was_found()                
        
        while self.target_position[0] > self.map.x[0] + 1 and not self.was_found:
            self.current = 'B'
            self.set_back()
            self.was_found = self.object_was_found()
        
        while self.target_position[1] + 1 <= self.map.y[1] and not self.was_found:
            self.current = 'L'
            self.set_left()
            self.was_found = self.object_was_found()
    
    def make_trajectory(self):
        self.foward_enable = True
        x=False
        count=0
        end=False
        while not self.was_found:
            self.was_found = self.object_was_found()
            if not self.was_found:
                if self.foward_enable:
                    self.current = 'F'
                    self.set_foward()
                    
                if self.back_enable:
                    self.current = 'B'
                    self.set_back()
                
                if self.shift_enable:
                    self.current = 'R'
                    count = 0
                    #This offset is equivalent perspective set for the vision sensor
                    #1.5 was used in this case because of defined size for the proximity sensor.
                    while count < 70:
                        if (self.target_position[1] -1.5 <= self.map.y[0] and 
                                self.target_position[0] -1.5 <= self.map.x[0]
                            ) or end or self.was_found:
                            break
                        
                        self.set_rigth()
                        self.was_found = self.object_was_found()
                        count = count + 1
                    self.shift_enable = False
                    if x:
                        self.back_enable = True
                    else:
                        self.foward_enable = True
                
                #1.5 was used in this case because of defined size for the proximity sensor.
                if self.target_position[0] + 1.5 >= self.map.x[1] and (count==0 or not x):
                    x=True
                    self.foward_enable = False
                    self.shift_enable = True
                    
                if self.target_position[0] -1.5 <= self.map.x[0] and x:
                    x=False
                    self.back_enable = False
                    self.shift_enable = True
                
                if self.target_position[0]+1.5 >= self.map.x[1] and self.target_position[1] -1.5 <= self.map.y[0]:
                    end = True
                
                if self.target_position[1] -1.5 <= self.map.y[0] and end and not x:
                    break
            else:
                break
      
    @property
    def target_position(self):
        err_code, position = vrep.simxGetObjectPosition(self.map.clientID,self.target,-1,vrep.simx_opmode_buffer)
        return position

    @property
    def get_proximity_sensor_1(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.map.clientID, self.proximity_sensor_1, vrep.simx_opmode_buffer
        )
        
        return state
    
    @property
    def get_proximity_sensor_2(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.map.clientID, self.proximity_sensor_2, vrep.simx_opmode_buffer
        )
        
        return state
    
    @property
    def get_proximity_sensor_3(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.map.clientID, self.proximity_sensor_3, vrep.simx_opmode_buffer
        )
        
        return state
    
    @property
    def get_proximity_sensor_4(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.map.clientID, self.proximity_sensor_4, vrep.simx_opmode_buffer
        )
        
        return state
    
    @property
    def get_proximity_sensor_5(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.map.clientID, self.proximity_sensor_5, vrep.simx_opmode_buffer
        )
        
        return state