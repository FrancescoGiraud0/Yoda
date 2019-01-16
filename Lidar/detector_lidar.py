
#!/usr/bin/env python3
'''Records measurments to a given file. Usage example:

$ ./record_measurments.py out.txt'''
import time
import sys
import config
from rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)


def find_obstacle(measure_list):
    measure_bool, measure_power = bool(measure_list[0]), int(measure_list[1])
    measure_angle, measure_distance = float(measure_list[2]), float(measure_list[3])
    ret_dict= {}
    
    if measure_distance <= config.MAX_DISTANCE and measure_distance >= config.MIN_DISTANCE and measure_power >= config.QUALITY:
        if measure_angle > config.ANGLE / 2 and measure_angle <= config.RIGHT:
            ret_dict["direction"] = "right"
            ret_dict["distance"] = measure_distance
        elif measure_angle >= config.LEFT and measure_angle<(config.LEFT+(config.ANGLE/2)):
            ret_dict["direction"] = "left"
            ret_dict["distance"] = measure_distance
        elif measure_angle >= config.CENTER and measure_angle <= config.ANGLE/2:
            ret_dict["direction"] = "center"
            ret_dict["distance"] = measure_distance
        elif measure_angle >= ((config.CENTER%360+360)-(config.ANGLE/2)) and measure_angle <= ((config.CENTER%360)+360):
            ret_dict["direction"] = "center"
            ret_dict["distance"] = measure_distance
    return ret_dict

def run():
    '''Main function'''
    try:
        print('Recording measurments... Press Crl+C to stop.')
        time.sleep(5)
        for measurment in lidar.iter_measurments(max_buf_meas = 202):
            measure_list = [str(v) for v in measurment]
            obstacle = find_obstacle(measure_list)
            #check if dictionary is not empty
            if len(obstacle)>0:
                print(obstacle)
    except KeyboardInterrupt:
        print('Stoping.')
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

if __name__ == '__main__':
    run()
