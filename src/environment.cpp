/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "kdtree.h"
#include <thread>
#include <unordered_set>

float A, B, C;

void clusterHelper(size_t indice, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::vector<size_t> &cluster,
                   std::vector<bool> &processed, KdTree *tree, float distanceTol);

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
euclideanCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, KdTree *tree, float distanceTol, int minSize,
                 int maxSize);

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
RansacPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int maxIterations, float distanceTol);

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  auto startTime = std::chrono::steady_clock::now();
  auto cc = pointProcessorI->FilterCloud(inputCloud, 0.2,
                                         Eigen::Vector4f(-20, -6, -3, 1), Eigen::Vector4f(30, 7, 2, 1));

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_cloud = RansacPlane(
    cc, 100, 0.2);

  auto *tree = new KdTree;

  for (size_t i = 0; i < segmented_cloud.first->points.size(); i++)
    tree->insert(segmented_cloud.first->points[i], i);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = euclideanCluster(segmented_cloud.first, tree, 0.35,
                                                                                int(A), int(B));

  int clusterId = 0;

  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
  cout << "cluster size" << clusters.size() << endl;
  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster : clusters) {
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

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
  int option = 0;
  while ((option = getopt(argc, argv, "a:b:c:")) != -1) {
    switch (option) {
      case 'a' :
        A = std::stof(std::string(optarg));
        break;
      case 'b' :
        B = std::stof(std::string(optarg));
        break;
      case 'c':
        C = std::stof(std::string(optarg));
        break;
      default:
        return EXIT_SUCCESS;
    }
  }
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  auto *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
  while (!viewer->wasStopped()) {

    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);
//    std::this_thread::sleep_for(std::chrono::milliseconds (50));
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

void clusterHelper(size_t indice, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::vector<size_t> &cluster,
                   std::vector<bool> &processed, KdTree *tree, float distanceTol) {
  processed[indice] = true;
  cluster.push_back(indice);

  std::vector<size_t> nearest = tree->search(cloud->points[indice], distanceTol);

  for (size_t id : nearest) {
    if (!processed[id])
      clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
  }
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
euclideanCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, KdTree *tree, float distanceTol, int minSize,
                 int maxSize) {
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  std::vector<bool> processed(cloud->points.size(), false);

  for (size_t idx = 0; idx < cloud->points.size(); ++idx) {
    if (!processed[idx]) {
      std::vector<size_t> cluster_idx;
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);

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

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
RansacPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int maxIterations, float distanceTol) {
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

      pcl::PointXYZI point = cloud->points[index];

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

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZI point = cloud->points[index];

    if (inliersResult.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  return std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>(
    cloudOutliers, cloudInliers);
}
