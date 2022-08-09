# Semantic Histogram Based Graph Matching for Real-Time Multi-Robot Global Localization in Large Scale Environment

### Related Publications:

Xiyue Guo, Junjie Hu, Junfeng Chen, Fuqin Deng, Tin Lun Lam, **Semantic Histogram Based Graph Matching for Real-Time Multi-Robot Global Localization in Large Scale Environment**, IEEE Robotics and Automation Letters, 2021. **[PDF](https://arxiv.org/pdf/2010.09297.pdf)**. 

<a href="https://www.youtube.com/watch?v=xB8WHj8K9cE" target="_blank"><img src="https://github.com/gxytcrc/Semantic-Graph-based--global-Localization/blob/main/example/fengmian.png" 
alt="ORB-SLAM3" width="240" height="150" border="10" /></a>

Results
-
![](https://github.com/gxytcrc/Semantic-Graph-based--global-Localization/blob/main/example/result1.png)
![](https://github.com/gxytcrc/Semantic-Graph-based--global-Localization/blob/main/example/result2.png)

# 1. Prerequisites #
* Ubuntu
* CMake
* Eigen
* Pangolin
* OpenCV
* PCL (version below 1.10)

# 2. Running #
Clone the repository and catkin_make:
```
    git clone https://https://github.com/gxytcrc/Semantic-Graph-based--global-Localization.git
    mkdir build
    cd build
    cmake ..
    make -j8
```
Download the dataset that is created from Airsim, and save them into the Datset . Download link: https://drive.google.com/file/d/106sPA48vFThLK0RB4WBcj-i8FZPQPmcV/view?usp=sharing.

Launch it as follows:
```
./mapAlignment robot1-foldername startFrameNumber endFrameNumber robot2-foldername startFrameNumber endFrameNumber
```
### Citation
```
@inproceedings{XiyueGuo2021,
title={Semantic Histogram Based Graph Matching for Real-Time Multi-Robot Global Localization in Large Scale Environment},
author={Xiyue Guo, Junjie Hu, Junfeng Chen, Fuqin Deng, Tin Lun Lam},
booktitle=IEEE Robotics and Automation Letters (RA-L)},
year={2021}
}
```

