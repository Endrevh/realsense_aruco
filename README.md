# SINTEF Manufacturing visual servo using realsense/ur_rtde/opencv/aruco

<div align="center">
    <img src="./logo.png" alt="SINTEF-logo" width="30%"  style="border-radius: 50%; padding-bottom: 20px"/>
</div>

### Dependencies 📋
Ensure you have the following installed, with their respective dependencies:
- OpenCV for C++, refer to docs (https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) for installation. Note that OpenCV needs to be built with extra modules to include the aruco module https://github.com/opencv/opencv_contrib
- UR_RTDE from SDU Robotics for C++
- RealSense SDK 2.0 for C++
- Eigen library for C++

### Installation and build 💽

- **Step 1**: 

Clone the repository using:

```bash
git clone https://github.com/Endrevh/realsense_aruco.git && cd build
  ```

- **Step 2**:
Run the cmake command
```bash
cmake .
```

- **Step 3**:
Run the make command
```bash
make
```

### Usage 🚀

After running make, ensure that the computer is properly connected to the UR10 CB3 control box and the Intel RealSense camera.
Then, run the executable with your selected user inputs. Currently, the project supports two types of controllers:

- **Servo control from ur_rtde library**
To use this controller, run the executable with the parameters on the following format:
```bash
./realsense_rtdi servo SF
```
where SF is a scaling factor between 0.0 and 1.0, where 1.0 represents the most aggressive control. E.g., for a scaling factor of 0.6, run:
```bash
./realsense_rtdi servo 0.6
```

- **Speed control**
This is a proportional speed controller with the tracking object's velocity as feedforward. I.e., the controller produces a system input which is a sum of (1) a constant parameter multiplied by the difference between current pose and desired pose and (2) a constant parameter multiplied by the estimated velocity of the object we want to track. To use this controller, run the executuable with paremeters on the following format:
```bash
./realsense_rtdi speed Kp FF
```
where Kp is the proportional constant and FF is the feedforward constant. Note that this controller can become unstable quickly, so start out with low values, e.g. Kp = 1.0 and FF = 0.1.
```bash
./realsense_rtdi speed 1.0 0.1
```








