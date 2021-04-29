# Lidar Obstacle Detection Project

In this project, lidar data stream is used to detect obstacles on the road. 3D ransac algorithm is implemented to segment pcd. Clustering data is done by implementing KD-tree in 3d. 


## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/shamse/SFND_Lidar_project.git
$> git checkout project
$> cd SFND_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make project
$> ./project
```


## Key Files

The key files for this project are 

* project.cpp
* environment.h
* src/quiz/ransac/ransac.h
* src/quiz/cluster/euclideanCluster.h
