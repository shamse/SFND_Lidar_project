/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0.0);

    // TODO:: Create point processor
    auto point_cloud {lidar->scan()};
    // renderRays(viewer, lidar->position, point_cloud);
    // renderPointCloud(viewer, point_cloud, "point cloud");

    // segment
    ProcessPointClouds<pcl::PointXYZ> point_processor;
    auto segmentCloud {point_processor.SegmentPlane(point_cloud, 100, 0.2)};

    // renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    // renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    // clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_processor.Clustering(segmentCloud.first, 2.0, 4, 30);

    int clusterID = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};

    for (const auto& cluster : cloudClusters)
    {
        std::cout << "Cluster size : " ;
        point_processor.numPoints(cluster);  // cout num points!
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterID), colors[clusterID]);
        // bounding box
        Box box = point_processor.BoundingBox(cluster);
        renderBox(viewer, box, clusterID);
        ++clusterID;
    }


    delete lidar;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer) 
{
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // render a test box to identify limits
    Box box1 {-1.5, -1.7 , -1., 2.6, 1.7, -0.4}; // same as roof crop
    Box box2 {-8, -8, -1, 12, 8, 1};
    renderBox(viewer, box1, 0, Color(1., 0., 1.));
    // renderBox(viewer, box2, 1, Color(1., 1., 0.));

    Eigen::Vector4f regionMin {-10, -6, -2, 1};
    Eigen::Vector4f regionMax {24, 6, 2, 1};
    float voxelSize = 0.25;

    // Experiment with the ? values and find what works best
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, voxelSize, regionMin, regionMax);
    // renderPointCloud(viewer, filterCloud, "filterCloud");

    // segment
    auto segmentCloud {pointProcessorI->SegmentPlane(filterCloud, 100, 0.2)};  // returns pair of clouds

    // test render
    // renderPointCloud(viewer, segmentCloud.first, "cars", Color(1., 0., 0.));
    renderPointCloud(viewer, segmentCloud.second, "road", Color(0., 1., 0.));

    // clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.35, 15, 400);

    int clusterID = 0;

    for (const auto& cluster : cloudClusters)
    {
        std::cout << "Cluster size : " ;
        pointProcessorI->numPoints(cluster);  // cout num points!
        Color color = Color((double) rand()/(RAND_MAX), (double) rand()/(RAND_MAX), (double) rand()/(RAND_MAX));
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterID), color);
        // bounding box
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterID+1);
        ++clusterID;
    }

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}