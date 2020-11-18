/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include "kdtree.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom) {
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);

  viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  return viewer;
}

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer);

void clusterHelper(size_t indice, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<size_t> cluster,
                   std::vector<bool> &processed, KdTree *tree, float distanceTol) {
  processed[indice] = true;
  cluster.push_back(indice);

  std::vector<size_t> nearest = tree->search(cloud->points[indice], distanceTol);

  for (size_t id : nearest) {
    if (!processed[id])
      clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
FilterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float filterRes,
            Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process

  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  /** Voxel grid filtering */
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloud_filtered);

  /** bounding box*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_region(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::CropBox<pcl::PointXYZI> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloud_filtered);
  region.filter(*cloud_region);

  std::vector<int> indices;

  pcl::CropBox<pcl::PointXYZI> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloud_region);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (int point: indices) {
    inliers->indices.push_back(point);
  }

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud_region);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_region);
  return cloud_region;
}


std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
euclideanCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, KdTree *tree, float distanceTol, int minSize,
                 int maxSize) {
  // TODO: Fill out this function to return list of indices for each cluster

  std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  std::vector<bool> processed(cloud->points.size(), false);

  for (size_t idx = 0; idx < cloud->points.size(); ++idx) {
    if (!processed[idx]) {
      std::vector<size_t> cluster_idx;
      typename pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);

      clusterHelper(idx, cloud, cluster_idx, processed, tree, distanceTol);

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

int main() {
  auto startTime = std::chrono::steady_clock::now();

  // Create viewer

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  std::string file = "/home/shivam/LIDAR_TEST/src/sensors/data/pcd/data_1/0000000000.pcd";
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI>(file, *inputCloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }
  std::cerr << "Loaded " << inputCloud->points.size() << " data points from " + file << std::endl;
  inputCloud = FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-20, -6, -3, 1), Eigen::Vector4f(30, 7, 2, 1));
  renderPointCloud(viewer, inputCloud, "inputCloud");

  auto *tree = new KdTree;

  for (size_t i = 0; i < inputCloud->points.size(); i++)
    tree->insert(inputCloud->points[i], i);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = euclideanCluster(inputCloud, tree, 0.35, 15, 500);

  int clusterId = 0;

  std::vector<Color> colors = { Color(1,0,0), Color(1,1,0), Color(0,0,1) };

  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : clusters)
  {
    std::cout << "cluster size ";
    pointProcessor->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId  % colors.size()]);

    Box box = pointProcessor->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);

    ++clusterId;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "pipeline took " << elapsedTime.count() << " milliseconds" << std::endl;

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }


}


void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {

  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
    case XY :
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    case TopDown :
      viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
      break;
    case Side :
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    case FPS :
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS)
    viewer->addCoordinateSystem(1.0);
}