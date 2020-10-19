# Semantic-Graph-based--global-Localization
Semantic graph based global localization for multi-robot map fusion. 

# 1. What do I need to build it? #
* Ubuntu
* CMake
* Eigen
* Pangolin
* OpenCV
* PCL

# 2. How do I use it? #
Clone the repository and catkin_make:
```
    git clone https://https://github.com/gxytcrc/Semantic-Graph-based--global-Localization.git
    mkdir build
    cd build
    cmake ..
    catkin_make
```
Download the dataset that is created from Airsim. Download link:  

Launch it as follows:
```
./mapAlignment robot1-foldername startFrameNumber endFrameNumber robot2-foldername startFrameNumber endFrameNumber
```
