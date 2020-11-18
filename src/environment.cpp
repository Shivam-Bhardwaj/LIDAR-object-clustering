// Create simple 3d highway environment using PCL
// for exploring self-driving car sensors

#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"
#include <thread>
#include <unordered_set>

#define FILTER_RESOLUTION 0.25

#define DISTANCE_TOL_RANSAC 0.2
#define MAX_RANSAC_ITER 60

#define DISTANCE_TOL_CLUSTERING 0.35
#define MIN_POINTS_IN_CLUSTER 10
#define MAX_POINTS_IN_CLUSTER 800

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  auto startTime = std::chrono::steady_clock::now();
  const auto MIN_POINT = Eigen::Vector4f(-15, -6, -3, 1);
  const auto MAX_POINT = Eigen::Vector4f(20, 7, 2, 1);
  auto filtered_cloud = pointProcessorI->FilterCloud(inputCloud, FILTER_RESOLUTION, MIN_POINT, MAX_POINT);

  auto segmented_cloud = pointProcessorI->RansacPlane(filtered_cloud, MAX_RANSAC_ITER, DISTANCE_TOL_RANSAC);

  auto *tree = new KdTree;

  for (size_t i = 0; i < segmented_cloud.first->points.size(); i++)
    tree->insert(segmented_cloud.first->points[i], i);

  auto clusters = pointProcessorI->EuclideanCluster(segmented_cloud.first, tree,
                                                    DISTANCE_TOL_CLUSTERING,
                                                    MIN_POINTS_IN_CLUSTER,
                                                    MAX_POINTS_IN_CLUSTER);

  int clusterId = 0;

  cout << "#clusters: " << clusters.size() << endl;
  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster : clusters) {
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), Color(1, 1, 1));
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }

  renderPointCloud(viewer, segmented_cloud.second, "planeCloud", Color(0, 1, 0));
  renderPointCloud(viewer, segmented_cloud.first, "obstCloud", Color(1, 0, 0));

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "pipeline took " << elapsedTime.count() << " milliseconds" << std::endl;
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer);

int main(int argc, char **argv) {
  std::cout << "starting environment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  auto *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
  while (!viewer->wasStopped()) {

    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);
    streamIterator++;
    if (streamIterator == stream.end())
      streamIterator = stream.begin();
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


