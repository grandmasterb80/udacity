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

    void addIndiciesOfTree( std::vector<int> &ids )
    {
        ids.push_back( id );
        if( left != nullptr ) left->addIndiciesOfTree( ids );
        if( right != nullptr ) right->addIndiciesOfTree( ids );
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

	void recSearch( Node *node,
                    std::vector<int> &ids,
                    std::vector<float> &target,
                    double distanceTol,
                    std::vector<float> &box_top_left,
                    std::vector<float> &box_bottom_right,
                    std::vector<float> &window_top_left,
                    std::vector<float> &window_bottom_right,
                    int axis,
                    int numMatches )
    {
        // step recursion with end reached
        if( node == nullptr ) return;

        assert( node->point.size() == box_top_left.size() );
        assert( node->point.size() == box_bottom_right.size() );
        assert( node->point.size() == window_top_left.size() );
        assert( node->point.size() == window_bottom_right.size() );

        // stop recursion when search window is completely in the box
        if( numMatches == node->point.size() )
        {
            // will add id of this node and all children to the ids
            node->addIndiciesOfTree( ids );
            return;
        }

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
        { // scope for left side
            int numMatchesLeft = numMatches;
            bool newMatch = ( window_top_left[ axis ] >= box_top_left[ axis ] &&
                              !(window_bottom_right[ axis ] <= box_bottom_right[ axis ]) &&
                              node->point[ axis ] <= box_bottom_right[ axis ] );
            float oldBoundRight = window_bottom_right[ axis ];
            window_bottom_right[ axis ] = node->point[ axis ];

            recSearch( node->left,
                       ids,
                       target,
                       distanceTol,
                       box_top_left,
                       box_bottom_right,
                       window_top_left,
                       window_bottom_right,
                       nextAxis,
                       newMatch ? ( numMatches + 1 ) : numMatches );

            // reset window to old value
            window_bottom_right[ axis ] = oldBoundRight;
        }

        { // scope for left side
            int numMatchesRight = numMatches;
            bool newMatch = ( !(window_top_left[ axis ] >= box_top_left[ axis ]) &&
                              node->point[ axis ] >= box_top_left[ axis ] &&
                              window_bottom_right[ axis ] <= box_bottom_right[ axis ] );
            float oldBoundLeft = window_top_left[ axis ];
            window_top_left[ axis ] = node->point[ axis ];

            recSearch( node->right,
                       ids,
                       target,
                       distanceTol,
                       box_top_left,
                       box_bottom_right,
                       window_top_left,
                       window_bottom_right,
                       nextAxis,
                       newMatch ? ( numMatches + 1 ) : numMatches );

            // reset window to old value
            window_top_left[ axis ] = oldBoundLeft;
        }
    }
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        int numMatches = 0;
        std::vector<float> box_top_left, box_bottom_right;
        std::vector<float> window_top_left, window_bottom_right;
        int i = 0;
        for( std::vector<float>::iterator t = target.begin(); t != target.end(); t++ )
        {
            box_top_left.push_back( (*t) - distanceTol );
            box_bottom_right.push_back( (*t) + distanceTol );
            window_top_left.push_back( -100.0 );
            window_bottom_right.push_back( 100.0 );
            if( window_top_left[ i ] >= box_top_left[ i ] && window_bottom_right[ i ] <= box_bottom_right[ i ] ) numMatches++;
            i++;
        }

        recSearch( root, ids, target, distanceTol, box_top_left, box_bottom_right, window_top_left, window_bottom_right, 0, numMatches );

		return ids;
	}
	

};




