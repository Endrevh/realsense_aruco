import sys, os, time


import rtde_control
import rtde_receive

  
import numpy as np

def main():


    state = 0
    tvec_init = None

    robot_ip = "192.168.0.90"

    print(robot_ip)
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    
    print(robot_ip)

    if rtde_r.isConnected():
        print(f"Connected to robot on {robot_ip}!")
    desired_velocity_vector = [0, 0, 0, 0, 0, 0]    
     






  

if __name__ == '__main__':
    main()
