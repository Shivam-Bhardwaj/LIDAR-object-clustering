# Playing with LIDAR data(PCD)

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### The Project

In this project, LIDAR's data is processed to cluster meaningful objects in the scene.

The Pipeline followed is:

1. The point cloud is processed to reduce the number of points to be processed.
2. Using a custom RANSAC implementation, the ground plane is filtered out from the point cloud data.
3. A k-d Tree is created from the points for clustering.
4. Euclidean clustering is performed on the k-d tree to get the clusters of objects. 
5. Display the clusters and the ground plane to render the image above.


## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/Shivam-Bhardwaj/LIDAR-object-clustering
$> cd LIDAR-object-clustering
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

The code was tested on the following specifications

- **CPU:** `Intel(R) Core(TM) i9-8950HK CPU @ 4.8 Ghz`
- **GPU:** `Nvidia GeForce GTX 1050 Ti Mobile`
- **OS:** `Ubuntu 16.04.6 LTS (Xenial Xerus)`
- **Kernal:** `4.15.0-48-generic`