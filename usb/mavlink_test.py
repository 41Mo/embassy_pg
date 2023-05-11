#!/usr/bin/env python3
from argparse import ArgumentParser
from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAVLink as mavlink
from pymavlink.dialects.v20 import ardupilotmega as mavcmd
from time import time_ns

test_dev = '/dev/serial/by-id/usb-Embassy_USB-serial_example_12345678-if00'

def wait_heartbeat(m:mavutil.mavudp):
    '''wait for a heartbeat so we know the target system IDs'''
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    if msg != None:
        print("SYSID= ", msg._msgbuf[3], "\nCOMPID= ", msg._msgbuf[4] , "\nPAYLOAD= ", msg)

def main():
    connection:mavutil.mavudp

    connection = mavutil.mavlink_connection(device=test_dev, baud=1000000, source_system=255, source_component=254)
    print("Waiting for APM heartbeat")
    # wait_heartbeat(connection)

    mav = connection.mav
    mav:mavlink
    send_time = 0
    recv_hb_time = 0
    recv_si_time = 0
    si_freq = 0
    si_packet = 0
    nloops = 0
    while(True):
        ct = time_ns()
        if (ct - send_time >= 1e9):
            mav.heartbeat_send(mavutil.ardupilotmega.MAV_TYPE_GCS, mavutil.ardupilotmega.MAV_AUTOPILOT_INVALID, 0,0,0)
            send_time = ct
        data = connection.recv_match(type="HEARTBEAT")
        if data is not None:
            freq = 1/(ct-recv_hb_time)*1e9
            recv_hb_time = ct
            print("HEARTBEAT_FREQ: ", freq)
            print("HEARTBEAT_PACKET: ", data)
            nloops = 0
            si_freq = 0

        data = connection.recv_match(type="SCALED_IMU")
        if data is not None:
            si_freq = 1/(ct - recv_si_time)*1e9
            recv_si_time = ct
            si_packet = data
            print("SCALED_IMU: ", si_freq)
            print("SCALED_IMU_PACKET: ", si_packet)
            # nloops+=1


    

if __name__ == "__main__":
    main()
