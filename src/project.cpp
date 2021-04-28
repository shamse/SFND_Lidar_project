/* author Ehsan Shams */
// LIDAR Obstacle Detection Project 
// Udacity - sensor fusion program

#include <iostream>
#include "environment.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "quiz/ransac/ransac.h"
#include "quiz/cluster/eculideanCluster.h"

template <typename PointT>
void cityBlock
(
    pcl::visualization::PCLVisualizer::Ptr& viewer, 
    ProcessPointClouds<PointT>* pointProcessorI, 
    const typename pcl::PointCloud<PointT>::Ptr& inputCloud
) 
{
    // render a test box to identify limits
    // Box box1 {-1.5, -1.7 , -1., 2.6, 1.7, -0.4}; // same as roof crop
    // Box box2 {-8, -8, -1, 12, 8, 1};
    // renderBox(viewer, box1, 0, Color(1., 0., 1.));
    // renderBox(viewer, box2, 1, Color(1., 1., 0.));

    Eigen::Vector4f regionMin {-10, -6, -2, 1};
    Eigen::Vector4f regionMax {24, 6, 2, 1};
    float voxelSize = .25;

    // Experiment with the ? values and find what works best
    typename pcl::PointCloud<PointT>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, voxelSize, regionMin, regionMax);
    // renderPointCloud(viewer, filterCloud, "filterCloud");

    // segment
    auto inliers = Ransac3D<PointT>(filterCloud, 100, 0.2);
    auto segmentCloud = separateClouds<PointT>(filterCloud, inliers);

    // test render
    renderPointCloud(viewer, segmentCloud.first, "road", Color(0., 1., 0.));
    // renderPointCloud(viewer, segmentCloud.second, "cars", Color(1., 0., 0.));

    // clustering
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters = Clustering<PointT, 3>(segmentCloud.second, 0.5);

    int clusterID = 0;

    for (const auto& cluster : cloudClusters)
    {
        std::cout << "Cluster size : " ;
        pointProcessorI->numPoints(cluster);  // cout num points!
        Color color = Color((double) rand()/(RAND_MAX), (double) rand()/(RAND_MAX), (double) rand()/(RAND_MAX));
        // Color color = Color(1.,0.,0.);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterID), color);
        // bounding box
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterID+1, Color(1., 0., 0.), 0.5);
        ++clusterID;
    }

}

int main(int argc, char** argv)
{
    // 
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // city block stream
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    while (!viewer->wasStopped ())
    {
        // clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load pcd and run obstacle detection process
        inputCloud = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloud);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 

    return 0;
}