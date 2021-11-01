# spin_dynamicCalibration

## Workspace
* Ubuntu 20.04 (Focal Fossa)

### Cmake:
__________________________________________
link: http.
*
*
### OpenCV:
__________________________________________
### SDU ur rtde:
__________________________________________
link: https://sdurobotics.gitlab.io/ur_rtde/index.html
* sudo add-apt-repository ppa:sdurobotics/ur-rtde
* sudo apt-get update
* sudo apt install librtde librtde-dev

Dependencies:
* sudo apt-get install libboost-all-dev

Build:
* git clone https://gitlab.com/sdurobotics/ur_rtde.git
* cd ur_rtde
* git submodule update --init --recursive
* mkdir build
* cd build
* cmake ..
* make
* sudo make install

### qt Crator:
__________________________________________

* sudo apt-get install qtcreator

### UR-Sim
__________________________________________
link: 
### pylon
__________________________________________
### ROS
__________________________________________

