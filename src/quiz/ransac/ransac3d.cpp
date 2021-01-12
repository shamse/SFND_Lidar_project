/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	while (maxIterations--)
	{
		// Randomly sample subset and fit line

		std::unordered_set<int> inliers{};

		while(inliers.size()<2)
			inliers.insert(rand()%cloud->points.size());

		auto itr{inliers.begin()};
		auto x1{cloud->points[*itr].x};
		auto y1{cloud->points[*itr].y};

		itr++;
		auto x2{cloud->points[*itr].x};
		auto y2{cloud->points[*itr].y};

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		auto a = y1 - y2;
		auto b = x2 - x1;
		auto c = x1*y2 - x2*y1;
		auto len = sqrt(a*a+b*b);

		if (len > 0.)
		{
			a /= len;
			b /= len;
			c /= len;
		}
		else
		{
			continue;
		}
		

		for (size_t idx = 0; idx < cloud->points.size(); idx++)
		{
			if (inliers.find(idx) == inliers.end())
			{
				auto x = cloud->points[idx].x;
				auto y = cloud->points[idx].y;
				auto dist = abs(a*x + b*y +c);
				if (dist < distanceTol)
					inliers.insert(idx);
			}
		}

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}

	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}


struct Plane
{
	pcl::PointXYZ p1, p2, p3;
	float A, B, C, D;

	Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::unordered_set<int> id)
	{
		assert(id.size() == 3);
		auto itr = id.begin();
		
		p1 = cloud->points[*(itr++)];
		p2 = cloud->points[*(itr++)];
		p3 = cloud->points[*(itr++)];

		Eigen::Vector3f v1{p2.x-p1.x, p2.y-p1.y, p2.z-p1.z};
		Eigen::Vector3f v2{p3.x-p1.x, p3.y-p1.y, p3.z-p1.z};
		auto v{v1.cross(v2)};
		v.normalize();
		A = v(0);
		B = v(1);
		C = v(2);
		D = -(A*p1.x+B*p1.y+C*p1.z);
	}

	auto distance(pcl::PointXYZ p)
	{
		return abs(A*p.x + B*p.y + C*p.z);
	}
};


auto Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxInteration, float distanceTol)
{
	std::unordered_set<int> result{};

	while (maxInteration--)
	{
		// pick random samples
		std::unordered_set<int> inliers{};
		while (inliers.size() < 3)
			inliers.insert(rand()%cloud->points.size());

		Plane plane(cloud, inliers);

		// loop over points and add to set if close
		for (int idx = 0; idx < cloud->points.size(); idx++)
		{
			if (inliers.find(idx) != inliers.end())
				continue;

			if (plane.distance(cloud->points[idx]) < distanceTol)
				inliers.insert(idx);
		}

		if (inliers.size() > result.size())
			result = inliers;
	}

	return result;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 100, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}