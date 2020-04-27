/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H
#define KDTREE_H

#include <cassert>
#include <sstream>
#include "../../render/render.h"

#define DO_SANITY_CHECKS

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

    void addIndiciesOfTree( std::vector<int> &ids )
    {
        ids.push_back( id );
        if( left != nullptr ) left->addIndiciesOfTree( ids );
        if( right != nullptr ) right->addIndiciesOfTree( ids );
    }

    std::string toStr()
    {
        std::stringstream ss;
        ss << "< id=" << id << ", ( ";
        bool x = false;
        for ( float pc : point )
        {
            if ( x ) ss << ",";
            ss << pc;
            x = true;
        }
        ss << ") >";
        return ss.str();
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
            int axis = 0;  // 0 = X, 1 = Y, 2 = ...
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

    void recSearch( Node *node, std::vector<int> &ids, int axis, const std::vector<float> &target, double distanceTol )
    {
        if( node == nullptr ) return;
        assert( node->point.size() == target.size() );

        double dd = 0.0;
        for( int i = 0; i < node->point.size(); i++ )
        {
            double d = node->point[ i ] - target[ i ];
            dd += d * d;
        }

        if( dd <= distanceTol * distanceTol )
        {
            ids.push_back( node->id );
        }

        int nextAxis = ( axis + 1 ) % node->point.size();

        if( target[ axis ] - distanceTol <= node->point[ axis ]  )
            recSearch( node->left, ids, nextAxis, target, distanceTol );
        if( target[ axis ] + distanceTol >= node->point[ axis ]  )
            recSearch( node->right, ids, nextAxis, target, distanceTol );
    }


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float> &target, float distanceTol)
	{
		std::vector<int> ids;
        recSearch( root, ids, 0, target, distanceTol );
		return ids;
	}
	

};

#endif // KDTREE_H
