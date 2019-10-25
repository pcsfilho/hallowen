import numpy as np
import vrep
import time

class Quadcopter(object):
    
    def __init__(self, clientID):
        self.clientID = clientID
        err_code, self.target = vrep.simxGetObjectHandle(self.clientID,'Quadricopter_target', vrep.simx_opmode_blocking)
        err_code, self.base = vrep.simxGetObjectHandle(self.clientID,"Quadricopter_base", vrep.simx_opmode_blocking)
        self.current_target_position = [0,0,0]
        self.proximity_sensor_1 = None
        self.proximity_sensor_2 = None
        self.proximity_sensor_3 = None
        self.proximity_sensor_4 = None
        self.proximity_sensor_5 = None
        self.shift = 0.05
            
    def init_sensors(self):    
        err_code, self.proximity_sensor_1 = vrep.simxGetObjectHandle(self.clientID,"s1", vrep.simx_opmode_blocking)
        err_code, self.proximity_sensor_2 = vrep.simxGetObjectHandle(self.clientID,"s2", vrep.simx_opmode_blocking)
        err_code, self.proximity_sensor_3 = vrep.simxGetObjectHandle(self.clientID,"s3", vrep.simx_opmode_blocking)
        err_code, self.proximity_sensor_4 = vrep.simxGetObjectHandle(self.clientID,"s4", vrep.simx_opmode_blocking)
        err_code, self.proximity_sensor_5 = vrep.simxGetObjectHandle(self.clientID,"s5", vrep.simx_opmode_blocking)

        vrep.simxReadProximitySensor(self.clientID, self.proximity_sensor_1, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.clientID, self.proximity_sensor_2, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.clientID, self.proximity_sensor_3, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.clientID, self.proximity_sensor_4, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.clientID, self.proximity_sensor_5, vrep.simx_opmode_streaming)
    
    def update_sensor(self, sensor):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.clientID, sensor, vrep.simx_opmode_buffer
        )
        return state
        
    def set_foward(self):
        err_code = vrep.simxSetObjectPosition(
            self.clientID,self.target, -1,
            [self.target_position[0]+self.shift, self.target_position[1], self.target_position[2]],
            vrep.simx_opmode_oneshot
        )
        
    def set_up(self):
        err_code = vrep.simxSetObjectPosition(
            self.clientID,self.target, -1,
            [self.target_position[0], self.target_position[1], self.target_position[2]+self.shift],
            vrep.simx_opmode_oneshot
        )
    
    def set_down(self):
        err_code = vrep.simxSetObjectPosition(
            self.clientID,self.target, -1,
            [self.target_position[0], self.target_position[1], self.target_position[2]-self.shift],
            vrep.simx_opmode_oneshot
        )
    
    def set_back(self):
        err_code = vrep.simxSetObjectPosition(
            self.clientID,self.target, -1,
            [self.target_position[0]-self.shift, self.target_position[1], self.target_position[2]],
            vrep.simx_opmode_oneshot
        )
    
    def set_foward(self):
        err_code = vrep.simxSetObjectPosition(
            self.clientID, self.target, -1,
            [self.target_position[0]+self.shift, self.target_position[1], self.target_position[2]],
            vrep.simx_opmode_oneshot
        )
        
    def fix_trajectory(self, direction):
        last_position = self.target_position
        time.sleep(0.05)
        if direction == 'foward':
            function = self.set_foward
            sensor = self.proximity_sensor_1
        else:
            function = self.set_back
            sensor = self.proximity_sensor_3
            
        while 1:
            if not self.update_sensor(sensor):
                count = 0
                time.sleep(0.05)
                while count <=50:
                    function()
                    count = count + 1
                    time.sleep(0.02)
                
                while self.target_position[2] > last_position[2]:
                    if self.get_proximity_sensor_5:
                        break
                    self.set_down()
                    time.sleep(0.05)
                break
            
            self.set_up()
            
    
    @property
    def target_position(self):
        err_code, self.current_target_position = vrep.simxGetObjectPosition(self.clientID,self.target,-1,vrep.simx_opmode_buffer)
        return self.current_target_position
    
    @property
    def get_proximity_sensor_1(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.clientID, self.proximity_sensor_1, vrep.simx_opmode_buffer
        )
        
        return state
    
    @property
    def get_proximity_sensor_2(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.clientID, self.proximity_sensor_2, vrep.simx_opmode_buffer
        )
        
        return state
    
    @property
    def get_proximity_sensor_3(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.clientID, self.proximity_sensor_3, vrep.simx_opmode_buffer
        )
        
        return state
    
    @property
    def get_proximity_sensor_4(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.clientID, self.proximity_sensor_4, vrep.simx_opmode_buffer
        )
        
        return state
    
    @property
    def get_proximity_sensor_5(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.clientID, self.proximity_sensor_5, vrep.simx_opmode_buffer
        )
        
        return state