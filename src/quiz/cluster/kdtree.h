/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template <int dim>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		Node* node = new Node(point, id);

		// insert(root, node);
		insertHelper(&root, 0, point, id);
	}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if (*node == nullptr)
		{
			*node = new Node(point, id);
		}
		else
		{
			int cd = depth%dim;

			if (point[cd] < (*node)->point[cd])
			{
				insertHelper(&((*node)->left), depth+1, point, id);
			}
			else
			{
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> res;

		searchHelper(res, root, 0, target, distanceTol);
		
		return res;
	}
	
	void searchHelper(std::vector<int>& res, Node* root, int depth, std::vector<float> target, float tol) 
	{
		if (!root)
			return;

		// bbox comparison
		float dx = abs(target[0]-root->point[0]);
		float dy = abs(target[1]-root->point[1]);
		float dz = abs(target[2]-root->point[2]);

		if ( dx < tol && dy < tol && dz < tol) {
			// radial comparison
			if (sqrt(dx*dx + dy*dy + dz*dz) < tol) {
				res.push_back(root->id);
			}
		}

		// move on to next point(s)
		int cd = depth%dim;
		
		if (target[cd] - tol < root->point[cd])
			searchHelper(res, root->left, depth+1, target, tol);
		
		if (target[cd] + tol > root->point[cd])
			searchHelper(res, root->right, depth+1, target, tol);

		return;
	}

};




