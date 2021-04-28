#ifndef EUCLIDEANCLUSTER_H
#define EUCLIDEANCLUSTER_H

#include "kdtree.h"

template <int dim>
void clusterHelper
(
    std::vector<int>& cluster, 
    std::vector<bool>& processed, 
    const std::vector<std::vector<float>>& points, 
    int id, 
    KdTree<dim>* tree, 
    float tol
) 
{
	processed[id] = true;
	cluster.push_back(id);
	
	std::vector<int> nearby_points {tree->search(points[id], tol)};

	for (int p : nearby_points) {
		if (!processed[p]) {
			clusterHelper(cluster, processed, points, p, tree, tol);
		}
	}
}

template <int dim>
std::vector<std::vector<int>> euclideanCluster
(
    const std::vector<std::vector<float>>& points, 
    KdTree<dim>* tree, 
    float distanceTol
)
{
	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(),false);

	for (int i = 0; i < points.size(); i++) {
		if (!processed[i]) {
			std::vector<int> cluster;
			clusterHelper(cluster, processed, points, i, tree, distanceTol);
			clusters.push_back(cluster);
		}
	}
 
	return clusters;

}

template<typename PointT, int dim>
std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering
(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float clusterTolerance
)
{
	KdTree<dim>* tree = new KdTree<dim>;

    std::vector<std::vector<float>> points;

    for (const auto& p : cloud->points) {
        points.push_back({p.x, p.y, p.z});
    }
  
    for (int i = 0; i < cloud->points.size(); i++) 
    	tree->insert(points[i],i); 

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//
  	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, clusterTolerance);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::vector<typename pcl::PointCloud<PointT>::Ptr> ans;

    for (const auto& c : clusters) {

        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

        for(int index: c) 
        {
            PointT p;
            p.x = points[index][0];
            p.y = points[index][1];
            p.z = points[index][2];
  			cloud->points.push_back(p);
        }

        ans.push_back(cloud);
    }

    return ans;
}

#endif