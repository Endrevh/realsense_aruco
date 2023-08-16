import sys, os, time


import cv2 as cv
import pyrealsense2 as rs

import rtde_control
import rtde_receive

  
import numpy as np
import matplotlib.pyplot as plt


def kalman_filter_continuous(measurement, state_estimate, state_covariance, process_noise_cov, measurement_noise_cov):
    # State transition matrix (6x6)
    F = np.array([[1, 0, 0, 1, 0, 0],   # Position and velocity transition matrix
                  [0, 1, 0, 0, 1, 0],
                  [0, 0, 1, 0, 0, 1],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]])

 

    # Observation matrix (we directly observe the position in 3D)
    H = np.array([[1, 0, 0, 0, 0, 0],  # Observing only x-coordinate in this example
                  [0, 1, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0]])

 

    # Predict Step
    # Project the state estimate forward
    predicted_state_estimate = F @ state_estimate

 

    # Project the error covariance forward
    predicted_state_covariance = F @ state_covariance @ F.T + process_noise_cov

 

    # Update Step
    # Calculate the Kalman gain
    kalman_gain = predicted_state_covariance @ H.T @ np.linalg.inv(H @ predicted_state_covariance @ H.T + measurement_noise_cov)

 

    # Update the state estimate based on the new measurement
    state_estimate = predicted_state_estimate + kalman_gain @ (measurement - H @ predicted_state_estimate)

 

    # Update the error covariance
    state_covariance = (np.eye(6) - kalman_gain @ H) @ predicted_state_covariance

 

    return state_estimate, state_covariance

def mid_button_click_event(event, x, y, flags, param):
    if event == cv.EVENT_MBUTTONDOWN:
        global is_clicked
        is_clicked = True  



class RealSenseCamera():
    def __init__(self, nickname='RealSense_cam'):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.nickname = nickname
        self.nicknames = []
        self.nicknames.append(self.nickname)
        #
    def start_grabbing(self):
        self.pipeline.start(self.config)
        #
    def get_color_data(self):
        all_data = self.pipeline.wait_for_frames() 
        color_frame = all_data.get_color_frame()
        if color_frame.is_frame():
            color_data = np.asanyarray(color_frame.get_data())
            color_grab_flag = True
        else:
            color_data = None
            color_grab_flag = False
        return color_data, color_grab_flag
        #   
    def close(self):
        self.pipeline.stop()       
      


def main():

    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

    cam = RealSenseCamera()

    cam.start_grabbing()
 
    cam_matrix = np.load(str('./src/realsense_calibration_params/' + cam.nickname + '_matrix.npy'))
    distor_vec = np.load(str('./src/realsense_calibration_params/' + cam.nickname + '_distor_vec.npy'))

    state = 0
    tvec_init = None

    robot_ip = "192.168.0.90"

    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    
    if rtde_r.isConnected():
        print(f"Connected to robot on {robot_ip}!")
    desired_velocity_vector = [0, 0, 0, 0, 0, 0]    
    

    while True:
        color_data, color_grab_flag = cam.get_color_data()

        gimg = cv.cvtColor(color_data, cv.COLOR_BGR2GRAY)

        
        corners, ids, rejected = detector.detectMarkers(gimg)

        
        
        if ids is not None:
            if len(ids) == 1:
              rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners[0], 0.1, cam_matrix, distor_vec)
              cv.drawFrameAxes(color_data, cam_matrix, distor_vec, rvec, tvec, 0.1)

        if ids is not None and state == 0:
            if len(ids) == 1:
                state = 1
                tvec_init = tvec
                initial_state_estimate = np.array([[0], [0], [0], [0], [0], [0]])  # Initial state estimate (x, y, z position and velocities)
                initial_state_covariance = np.eye(6)  # Initial error covariance
                time_steps = []
                x = []  # Estimated x position
                y = []  # Estimated y position
                z = []  # Estimated z position
                x_mes = []  # Example x measurement (for demonstration purposes)
                y_mes = []  # Example y measurement (for demonstration purposes)
                z_mes = []  # Example z measurement (for demonstration purposes)
                vx = []
                vy = []
                vz = []
                print('State: ', state)
        elif ids is None and state == 1:
            state = 0  
            tvec_init = None
            desired_velocity_vector = [0, 0, 0, 0, 0, 0]
            print('State: ', state)

        if ids is not None and tvec_init is not None:
            if len(ids) == 1:
                pos_error =  tvec - tvec_init 
                posX_err = pos_error[0][0][0]
                posY_err = pos_error[0][0][1]
                posZ_err = pos_error[0][0][2]

                measurement = np.array([[posX_err], [posY_err], [posZ_err]])  # Example measurement (x, y, z coordinates)
                process_noise_cov = np.eye(6) * 0.1
                measurement_noise_cov = np.eye(3) * 0.2
                state_estimate, state_covariance = kalman_filter_continuous(measurement, initial_state_estimate, initial_state_covariance, process_noise_cov, measurement_noise_cov)
                initial_state_estimate = state_estimate
                initial_state_covariance = state_covariance
                


                if len(time_steps) == 0:
                    time_steps.append(1)
                else:    
                    time_steps.append(time_steps[-1] + 1)
                x.append(state_estimate[0, 0])  # Extract the estimated x position
                y.append(state_estimate[1, 0])  # Extract the estimated y position
                z.append(state_estimate[2, 0])  # Extract the estimated z position
                x_mes.append(measurement[0, 0])  # Store the example x measurement (for demonstration purposes)
                y_mes.append(measurement[1, 0])  # Store the example y measurement (for demonstration purposes)
                z_mes.append(measurement[2, 0])  # Store the example z measurement (for demonstration purposes)
                vx.append(state_estimate[3, 0]) 
                vy.append(state_estimate[4, 0])  
                vz.append(state_estimate[5, 0]) 

                c_vx = state_estimate[3, 0]
                c_vy = state_estimate[4, 0]
                c_vz = state_estimate[5, 0]

                #print(c_vx, c_vy, c_vz)

                
                if abs(c_vx) < 0.001 or abs(posX_err) < 0.05:
                    c_vx = 0
                    posX_err = 0
                if abs(c_vy) < 0.001 or abs(posY_err) < 0.05:
                    c_vy = 0
                    posY_err = 0
                if abs(c_vz) < 0.001 or abs(posZ_err) < 0.05:
                    c_vz = 0  
                    posZ_err = 0

                kp = 0.7
                kd = 6
                x_cont =  kp * posX_err + kd * c_vx
                y_cont =  kp * posY_err + kd * c_vy
                z_cont =  kp * posZ_err + kd * c_vz

                desired_velocity_vector = [x_cont, z_cont, -y_cont, 0, 0, 0]
                print('kpkd: ', kp * posZ_err, kd * c_vz)
                print('des vec: ', desired_velocity_vector)


                if len(time_steps) > 50:
                    time_steps.pop(0)
                    x.pop(0)
                    y.pop(0)
                    z.pop(0)
                    x_mes.pop(0)
                    y_mes.pop(0)
                    z_mes.pop(0)
                    vx.pop(0)
                    vy.pop(0)
                    vz.pop(0)

                #print('State is: ', state_estimate.T)    
                plt.clf() 
                plt.plot(time_steps, x, label='Estimated Position', color='red') 
                plt.plot(time_steps, y, label='Estimated Position', color='green') 
                plt.plot(time_steps, z, label='Estimated Position', color='blue') 
                plt.plot(time_steps, x_mes, label='Measurement', color='red', linestyle='dashed') 
                plt.plot(time_steps, y_mes, label='Measurement', color='green', linestyle='dashed') 
                plt.plot(time_steps, z_mes, label='Measurement', color='blue', linestyle='dashed') 
                plt.plot(time_steps, vx, label='Velocity', color='red', linestyle='dotted') 
                plt.plot(time_steps, vy, label='Velocity', color='green', linestyle='dotted') 
                plt.plot(time_steps, vz, label='Velocity', color='blue', linestyle='dotted') 
                plt.xlabel('Time Steps') 
                plt.ylabel('Position') 
                plt.title('Continuous Plot of State Estimation') 
                plt.legend(loc='upper right') 
                plt.grid(True) 
                plt.pause(0.001) 


        rtde_c.speedL(desired_velocity_vector, 0.1) 

        cv.aruco.drawDetectedMarkers(color_data, corners, ids)

        cv.imshow('Stream', color_data)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cv.destroyAllWindows()
    rtde_c.speedL([0, 0, 0, 0, 0, 0], 0.1) 
    rtde_c.stopScript()
    cam.close()

   






  

if __name__ == '__main__':
    main()
