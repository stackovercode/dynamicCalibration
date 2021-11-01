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
Link: https://www.osradar.com/install-qtcreator-on-ubuntu-20-04-18-04/
* sudo apt-get install qtcreator
* sudo apt install build-essential

Dependencies:<br/>
Link: https://askubuntu.com/questions/1232969/qtcreator-not-recognizing-linked-libraries-after-upgrading-ubuntu-to-20-04
* sudo apt install clang-8
* sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-8 100
* sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-8 100

### UR-Sim
__________________________________________
link: 
### pylon
__________________________________________
### ROS
__________________________________________

