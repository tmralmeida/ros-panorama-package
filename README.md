# data_matrix_detection
This package is a set of ROS nodes that allow a real-time data matrix detection.

### Prerequisites

What things that are needed:

* [ROS-melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) - The ROS version used.

Hardware:
* [Jetson Agx Xavier](https://developer.nvidia.com/embedded/jetson-agx-xavier-developer-kit) - Board
* [e-CAM130_CUXVR - Multiple Camera Board for NVIDIA® Jetson AGX Xavier™](https://www.e-consystems.com/nvidia-cameras/jetson-agx-xavier-cameras/four-synchronized-4k-cameras.asp) - Cameras board


### Project Guide

After all the prerequisites are installed, the first step is to calibrate the cameras in order to get the intrinsics parameters. This was based in this [Tutorial](http://wiki.ros.org/camera_calibration).

To launch the entire system run:

```
roslaunch data_matrix_detection bringup.launch
```



