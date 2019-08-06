# inspector_software_uav

ROS-based software for PV plants inspections. The project is divided into ground an earial segments.

This package contains the aerial segment, to be installed on each UAV on-board computer.

You can find the ground segment [here](http://olaf.grupotsk.com:8080/inspector/us/inspector_gcs.git)

## How to build and install inspector_software_uav

### ROS 

This package is based on ROS Kinetic Kame.

See instructions and install "Desktop-Full" package:

http://wiki.ros.org/kinetic/Installation/Ubuntu

### DJI_SDK & UAL

Fist of all, in order to use DJI autopilots, you need to install the DJI SDK for ROS. You can follow the next tutorial:

http://wiki.ros.org/dji_sdk/Tutorials/Getting%20Started

The next step is installing UAL for DJI autopilots. Clone the next repository and follow the instructions for installation:

https://github.com/AlejandroCastillejo/grvc-ual/tree/dji_1.0

### Other dependencies 

 * usb_cam driver:
    ```
    sudo apt-get install ros-kinetic-usb-cam
    ```

 * gphoto2 for python (for RGB cameras)
    ```
    sudo pip install gphoto2
    ```

 * sf11 altitude sensor driver
    ```
    cd ~/(your catkin_ws)/src
    git clone https://github.com/wavelab/cereal_port.git
    git clone https://github.com/AlejandroCastillejo/sf11_altitude_sensor.git
    ```

 * scp (transfer protocol for file uploading)
    ```
    sudo pip install scp
    sudo apt install sshpass
    ```
 
 * python-pandas (requiered for data bases)
    ```
    sudo pip install pandas
    ```
