// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
                                        Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process


  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  /** Voxel grid filtering */
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloud_filtered);

  /** bounding box*/
  typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloud_filtered);
  region.filter(*cloud_region);

  std::vector<int> indices;

  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloud_region);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (int point: indices) {
    inliers->indices.push_back(point);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_region);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_region);
  return cloud_region;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) {

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

  return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

  std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
                                             boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::RansacPlane(const typename pcl::PointCloud<PointT>::Ptr &cloud, int maxIterations,
                                        float distanceTol) {
  // Time segmentation process
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  while (maxIterations--) {
    std::unordered_set<int> inliers;

    while (inliers.size() < 3)
      inliers.insert(rand() % (cloud->points.size()));

    float x1, y1, z1, x2, y2, z2, x3, y3, z3;

    auto iter = inliers.begin();

    x1 = cloud->points[*iter].x;
    y1 = cloud->points[*iter].y;
    z1 = cloud->points[*iter].z;

    iter++;

    x2 = cloud->points[*iter].x;
    y2 = cloud->points[*iter].y;
    z2 = cloud->points[*iter].z;

    iter++;

    x3 = cloud->points[*iter].x;
    y3 = cloud->points[*iter].y;
    z3 = cloud->points[*iter].z;

    float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float d = -(a * x1 + b * y1 + c * z1);

    for (int index = 0; index < cloud->points.size(); index++) {
      if (inliers.count(index) > 0)
        continue;

      PointT point = cloud->points[index];

      float x4 = point.x;
      float y4 = point.y;
      float z4 = point.z;

      float dist = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);

      if (dist <= distanceTol)
        inliers.insert(index);
    }

    if (inliers.size() > inliersResult.size())
      inliersResult = inliers;
  }

  typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

  for (int index = 0; index < cloud->points.size(); index++) {
    PointT point = cloud->points[index];

    if (inliersResult.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloudOutliers,
                                                                                                 cloudInliers);
}

template<typename PointT>
void ProcessPointClouds<PointT>::ClusterHelper(size_t i, const typename pcl::PointCloud<PointT>::Ptr &cloud,
                                               std::vector<size_t> &cluster, std::vector<bool> &processed, KdTree *tree,
                                               float distanceTol) {

  processed[i] = true;
  cluster.push_back(i);

  std::vector<size_t> nearest = tree->search(cloud->points[i], distanceTol);

  for (size_t id : nearest) {
    if (!processed[id])
      ClusterHelper(id, cloud, cluster, processed, tree, distanceTol);
  }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::EuclideanCluster(const typename pcl::PointCloud<PointT>::Ptr &cloud, KdTree *tree,
                                             float distanceTol, int minSize, int maxSize) {

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  std::vector<bool> processed(cloud->points.size(), false);

  for (size_t idx = 0; idx < cloud->points.size(); ++idx) {
    if (!processed[idx]) {
      std::vector<size_t> cluster_idx;
      typename pcl::PointCloud<PointT>::Ptr cluster(new typename pcl::PointCloud<PointT>);

      ClusterHelper(idx, cloud, cluster_idx, processed, tree, distanceTol);

      if (cluster_idx.size() >= minSize && cluster_idx.size() <= maxSize) {
        for (size_t i : cluster_idx) {
          cluster->points.push_back(cloud->points[i]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;

        clusters.push_back(cluster);
      } else {
        for (size_t i = 1; i < cluster_idx.size(); i++) {
          processed[cluster_idx[i]] = false;
        }
      }
    }
  }
  return clusters;
}