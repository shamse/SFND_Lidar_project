
#include "../../render/render.h"
#include <unordered_set>

template <typename PointT>
struct Plane
{
	PointT p1, p2, p3;
	float A, B, C, D;

	Plane(typename pcl::PointCloud<PointT>::Ptr cloud, std::unordered_set<int> id)
	{
		assert(id.size() == 3);
		auto itr = id.begin();
		
		p1 = cloud->points[*(itr++)];
		p2 = cloud->points[*(itr++)];
		p3 = cloud->points[*itr];

		Eigen::Vector3f v1{p2.x-p1.x, p2.y-p1.y, p2.z-p1.z};
		Eigen::Vector3f v2{p3.x-p1.x, p3.y-p1.y, p3.z-p1.z};
		auto v{v1.cross(v2)};
		A = v(0);
		B = v(1);
		C = v(2);
		D = -(A*p1.x+B*p1.y+C*p1.z);
        float len = v.norm();
        A /= len;
        B /= len;
        C /= len;
        D /= len;
	}

	auto distance(PointT p)
	{
		return abs(A*p.x + B*p.y + C*p.z + D);
	}
};

template <typename PointT>
auto Ransac3D
(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    int maxIteration, 
    float distanceTol
)
{
	auto startTime = std::chrono::steady_clock::now();

    /* initialize random seed: */
    srand (time(NULL));


	std::unordered_set<int> result{};

	while (maxIteration--)
	{
		// pick random samples
		std::unordered_set<int> inliers{};
		while (inliers.size() < 3)
			inliers.insert(rand()%cloud->points.size());

		Plane<PointT> plane(cloud, inliers);

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

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac3D took " << elapsedTime.count() << " milliseconds" << std::endl;

	return result;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
separateClouds
(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    std::unordered_set<int> inliers
)
{
    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    return std::make_pair(cloudInliers, cloudOutliers);
}