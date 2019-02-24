import smbus            #import SMBus module of I2C
import time         
import collections
import numpy as np
import csv
import sys
import config
import socket

def MPU_Init():
        #write to sample rate register
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
        
        #Write to power management register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
        
        #Write to Configuration register
        bus.write_byte_data(Device_Address, CONFIG, 0)
        
        #Write to Gyro configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
        
        #Write to interrupt enable register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
        
    #concatenate higher and lower value
    value = ((high << 8) | low)
            
    #to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value


def SpeedTracker():

    #some MPU6050 Registers and their Address
    PWR_MGMT_1   = config.PWR_MGMT_1
    SMPLRT_DIV   = config.SMPLRT_DIV
    CONFIG       = config.CONFIG
    GYRO_CONFIG  = config.GYRO_CONFIG
    INT_ENABLE   = config.INT_ENABLE
    ACCEL_XOUT_H = config.ACCEL_XOUT_H
    ACCEL_YOUT_H = config.ACCEL_YOUT_H
    ACCEL_ZOUT_H = config.ACCEL_ZOUT_H
    GYRO_XOUT_H  = config.GYRO_XOUT_H
    GYRO_YOUT_H  = config.GYRO_YOUT_H
    GYRO_ZOUT_H  = config.GYRO_ZOUT_H

    bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
    Device_Address = config.Device_Address   # MPU6050 device address


    MPU_Init()

    tCycle = config.tCycle
    nCalibrationCycle=config.nCalibrationCycle
    nConvertX=config.nConvertX
    nConvertY=config.nConvertY
    nConvertZ=config.nConvertZ
    #while di calibrazione
    counter = 0
    #inizializzazione variabili

    averageAccX=0.0
    averageAccY=0.0
    averageAccZ=0.0

    while counter < nCalibrationCycle:
        #print 'Reading Data of Gyroscope and Accelerometer'
        #Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        averageAccX=averageAccX+acc_x
        averageAccY=averageAccY+acc_y
        averageAccZ=averageAccZ+acc_z
        
        counter=counter+1
        time.sleep(tCycle)
        
    averageAccX=averageAccX/config.nCalibrationCycle
    averageAccY=averageAccY/config.nCalibrationCycle
    averageAccZ=averageAccZ/config.nCalibrationCycle

    Vx=0.0
    Vy=0.0
    Vz=0.0
    spazioPercorsoX=0.0
    spazioPercorsoY=0.0
    spazioPercorsoZ=0.0

    while True:
        #Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
        
        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = (acc_x-averageAccX)/16384.0
        Vx = Ax*config.tCycle
        
        spazioPercorsoX=Vx*config.tCycle+spazioPercorsoX

        Ay = (acc_y-averageAccY)/16384.0
        Vy = Ay*config.tCycle
        
        spazioPercorsoY=Vy*config.tCycle+spazioPercorsoY

        Az = (acc_z-averageAccZ)/16384.0
        Vz = Az*config.tCycle
        
        spazioPercorsoZ=Vz*config.tCycle+spazioPercorsoZ

        time.sleep(tCycle)
    
        return Ax*nConvertX,Vx*nConvertX,Ay*nConvertY,Vy*nConvertY,Az*nConvertZ,Vz*nConvertZ
