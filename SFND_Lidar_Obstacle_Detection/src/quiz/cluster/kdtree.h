/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <cassert>
#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(const std::vector<float> &arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
    {
         // will recursively delete all leaves
        if( left != nullptr ) delete left;
        if( right != nullptr ) delete right;
    }
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
    {
        if( root != nullptr ) delete root; // will recursively delete all leaves
    }

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly within the root 
        uint32_t numAxes = point.size();
        Node* newNode = new Node( point, id );
        
        if( root == nullptr )
        {
            root = newNode;
        }
        else
        {
            Node* currentNode = root;
            bool axis = 0;  // 0 = X, 1 = Y, 2 = ...
            bool inserted = false;
            do {
                assert( currentNode->point.size() == numAxes );

                if( point[ axis ] < currentNode->point[ axis ] )
                {
                    if( currentNode->left == nullptr )
                    {
                        currentNode->left = newNode;
                        inserted = true;
                    }
                    else
                    {
                        currentNode = currentNode->left;
                    }
                }
                else
                {
                    if( currentNode->right == nullptr )
                    {
                        currentNode->right = newNode;
                        inserted = true;
                    }
                    else
                    {
                        currentNode = currentNode->right;
                    }
                }

                axis = ( axis + 1 ) % numAxes;
            } while( !inserted );
        }
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




