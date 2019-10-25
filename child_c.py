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
    while 1:
        #sim.get_position(quad.target, vrep.simx_opmode_buffer)
        quad.update_sensor(quad.proximity_sensor_1)
        quad.update_sensor(quad.proximity_sensor_2)
        quad.update_sensor(quad.proximity_sensor_3)
        quad.update_sensor(quad.proximity_sensor_4)
        quad.update_sensor(quad.proximity_sensor_5)
        
        # while quad.target_position[2] < sim.z_limit[1]:
        #     quad.set_up()
            
        # while quad.target_position[0] > sim.x_limit[0]:
        #     quad.set_back()
            
        if sim.START:
            
            quad.test_colision()
                    
            if quad.target_position[0] > sim.x_limit[0]:
                quad.set_foward()

            elif quad.target_position[1] > sim.y_limit[0]:
                quad.set_rigth()
            

            if (quad.target_position[0] <= sim.x_limit[0] and
                quad.target_position[1] <= sim.y_limit[0] and
                quad.target_position[2] >= sim.z_limit[1]
            ):
                sim.START = False
                sim.SEARCH = True
        
        # if (SEARCH):
        #     x = pos[0]
        #     y = pos[1]
        #     z = pos[2]
        #     if x_enable:
        #         print('X ENABLED')
        #         if x < x_limit[0] and vx_search < 0:
        #             vx_search = -vx_search
        #             x_enable = False
        #             y_enable = True
                
        #         if (x > x_limit[1] and vx_search > 0):
        #             vx_search =  - vx_search
        #             x_enable = False
        #             y_enable = True
                
                
        #         if (x > -6 and x < 6 and not boss):
        #             boss = True
        #             vx_search = vx_search * 2.5
        #         else:
        #             if (x <= -6 or x >= 6 and boss):
        #                 boss = False
        #                 vx_search = 0.1 if vx_search > 0 else -0.1
                    
        #         x = x + vx_search
            
        #     if y_enable:
        #         print('Y ENABLED')
        #         y = y + vy_search
        #         y_control = y_control + 1
                
        #         if y_control >= 50:
        #             y_control = 0
        #             y_enable = False
        #             x_enable = True
            
        #     print(f'x[{x}] y[{y}] z[{z}]')
        #     err_code = vrep.simxSetObjectPosition(clientID,targetObj,-1,[x, y, z],vrep.simx_opmode_oneshot)
        
        if (sim.SEARCH and sim.x >= 8 and sim.y >= 8):
            sim.SEARCH = False
                        
        time.sleep(0.2)
else:
    print("Not connected to remote A")

# vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
# print("Done")
