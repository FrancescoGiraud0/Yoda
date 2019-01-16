import time
import sys
import config
from rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)

def update_obstacles(obstacles, ob):
    for key in ob.keys():
        if ob[key] == True:
            print(ob.items())
            obstacles[key] = True
    return obstacles

def find_obstacles(measurements_list):
    
    ret_dict= {"left": False, "center": False, "right": False}
    
    for measure_tuple in measurements_list:
        
        ob_dict = {"left": False, "center": False, "right": False}
        
        measure_bool, measure_power, measure_angle, measure_distance = measure_tuple
        
        if measure_distance <= config.MAX_DISTANCE and measure_distance >= config.MIN_DISTANCE and measure_power >= config.QUALITY:
            #right
            if measure_angle > config.ANGLE / 2 and measure_angle <= config.RIGHT:
                ob_dict["right"] = True
            #left
            elif measure_angle >= config.LEFT and measure_angle<(config.LEFT+(config.ANGLE/2)):
                ob_dict["left"] = True
            #center
            elif measure_angle >= config.CENTER and measure_angle <= config.ANGLE/2:
                ob_dict["center"] = True
            #center
            elif measure_angle >= ((config.CENTER%360+360)-(config.ANGLE/2)) and measure_angle <= ((config.CENTER%360)+360):
                ob_dict["center"] = True
        
            ret_dict = update_obstacles(ret_dict, ob_dict)
        
    return ret_dict

def run():
    '''Main function'''
    try:
        time.sleep(5)
        measurments_list = []
        for measurment in lidar.iter_measurments(max_buf_meas = config.MAX_BUF_MEAS):
            measurments_list.append(measurment)
            if len(measurments_list) >= config.NUMBER_MEASURE:
                obstacles = find_obstacles(measurments_list)
                print(obstacles)
                obstacles.clear()
                measurments_list.clear()
    except KeyboardInterrupt:
        print('Stoping.')
        
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()


run()

