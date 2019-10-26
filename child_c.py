import vrep
import time

from helper import Simulation
from quadcopeter import Quadcopter


sim = Simulation()
sim.start_connection('127.0.0.1',19999)


if sim.clientID != -1:
    print ("Connected to remote API server")
    quad = Quadcopter(sim.clientID)
    
    quad.init_sensors()
    
    sim.get_position(quad.target, vrep.simx_opmode_streaming)
        
    time.sleep(0.2)

    #sim.get_position(quad.target, vrep.simx_opmode_buffer)
    quad.update_sensor(quad.proximity_sensor_1)
    quad.update_sensor(quad.proximity_sensor_2)
    quad.update_sensor(quad.proximity_sensor_3)
    quad.update_sensor(quad.proximity_sensor_4)
    quad.update_sensor(quad.proximity_sensor_5)
    
    while quad.target_position[2] <= sim.z_limit[1]:
        quad.set_up()
        
    while quad.target_position[0] > sim.x_limit[0] + 1:
        quad.current = 'B'
        quad.set_back()
    
    while quad.target_position[1] + 1 <= sim.y_limit[1]:
        quad.current = 'L'
        quad.set_left()
    
    quad.foward_enable = True
    x=False
    count=0
    end=False
    while 1:
        if quad.foward_enable:
            quad.current = 'F'
            quad.set_foward()
            
        if quad.back_enable:
            quad.current = 'B'
            quad.set_back()
        
        if quad.shift_enable:
            quad.current = 'R'
            count = 0
            while count < 75:
                if (quad.target_position[1] -1.5 <= sim.y_limit[0] and quad.target_position[0] -1.5 <= sim.x_limit[0]) or end:
                    break
                
                quad.set_rigth()
                count = count + 1
            quad.shift_enable = False
            if x:
                quad.back_enable = True
            else:
                quad.foward_enable = True
        
        if quad.target_position[0] + 1.5 >= sim.x_limit[1] and (count==0 or not x):
            x=True
            quad.foward_enable = False
            quad.shift_enable = True
            
        if quad.target_position[0] -1.5 <= sim.x_limit[0] and x:
            x=False
            quad.back_enable = False
            quad.shift_enable = True
        
        if quad.target_position[0]+1.5 >= sim.x_limit[1] and quad.target_position[1] -1.5 <= sim.y_limit[0]:
            print('*******************************END***************************')
            end = True
        
        if quad.target_position[1] -1.5 <= sim.y_limit[0] and end and not x:
            break
        
    print('Done')
               
else:
    print("Not connected to remote A")

# vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
# print("Done")
