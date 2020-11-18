// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>
#include "kdtree.h"

template<typename PointT>
class ProcessPointClouds {
public:

  //constructor
  ProcessPointClouds();

  //deconstructor
  ~ProcessPointClouds();

  void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

  typename pcl::PointCloud<PointT>::Ptr
  FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint,
              Eigen::Vector4f maxPoint);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
  RansacPlane(const typename pcl::PointCloud<PointT>::Ptr &cloud, int maxIterations, float distanceTol);

  Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

  void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

  typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

  std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

  void ClusterHelper(size_t indice, const typename pcl::PointCloud<PointT>::Ptr &cloud, std::vector<size_t> &cluster,
                     std::vector<bool> &processed, KdTree *tree, float distanceTol);

  std::vector<typename pcl::PointCloud<PointT>::Ptr>
  EuclideanCluster(const typename pcl::PointCloud<PointT>::Ptr &cloud, KdTree *tree, float distanceTol, int minSize,
                   int maxSize);

};

#endif /* PROCESSPOINTCLOUDS_H_ */