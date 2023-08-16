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




