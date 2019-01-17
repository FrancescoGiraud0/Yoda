#!/usr/bin/env python3
'''Records measurments to a given file. Usage example:

$ ./record_measurments.py out.txt'''
import time
import sys
from rplidar import RPLidar


PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)

#Lidar detection parameters
CENTER = 0
ANGLE = 60
LEFT = 60
RIGHT = 300
MIN_DISTANCE = 100
MAX_DISTANCE = 300


def find_obstacle(measure_list):
    measure_bool, measure_power = bool(measure_list[0]), int(measure_list[1])
    measure_angle, measure_distance = float(measure_list[2]), float(measure_list[3])
    ret_dict= {}
    
    if measure_distance <= MAX_DISTANCE and measure_distance >= MIN_DISTANCE and measure_power >= 13:
        if measure_angle in range(CENTER%360, LEFT):
            ret_dict["direction"] = "left"
            ret_dict["distance"] = measure_distance
        elif measure_angle in range(RIGHT , CENTER%360 + 360):
            ret_dict["direction"] = "right"
            ret_dict["distance"] = measure_distance
        
    return ret_dict

def run(path):
    '''Main function'''
    outfile = open(path, 'w')
    try:
        print('Recording measurments... Press Crl+C to stop.')
        time.sleep(5)
        for measurment in lidar.iter_measurments(max_buf_meas = 202):
            measure_list = [str(v) for v in measurment]
            obstacle = find_obstacle(measure_list)
            #check if dictionary is not empty
            if len(obstacle)>0:
                print(obstacle)
                line = '\t'.join(str(v) for v in measurment)
                outfile.write(line + '\n')
    except KeyboardInterrupt:
        print('Stoping.')
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    outfile.close()

if __name__ == '__main__':
    run(sys.argv[1])
