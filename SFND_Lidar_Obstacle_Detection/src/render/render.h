/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#ifndef RENDER_H
#define RENDER_H
#include <pcl/visualization/pcl_visualizer.h>
#include "box.h"
#include <iostream>
#include <string>
#include <utility>
#include <vector>

struct Color
{

	float r, g, b;

	Color(float setR, float setG, float setB)
		: r(setR), g(setG), b(setB)
	{}
};

struct Vect3
{

	double x, y, z;

	Vect3(double setX, double setY, double setZ)
		: x(setX), y(setY), z(setZ)
	{}

	Vect3(const Vect3& p)
		: x( p.x ), y( p.y ), z( p.z )
    {}

	Vect3()
		: x( 0.0 ), y( 0.0 ), z( 0.0 )
    {}

	Vect3 operator+(const Vect3& vec);
	Vect3 operator-(const Vect3& vec);
	Vect3 operator*(const Vect3& vec);
	double dot(const Vect3& vec);
	Vect3 operator*(const double a);
};

Vect3 operator*(const double a, const Vect3& vec);
Vect3 operator+(const Vect3& vec1, const Vect3& vec2);
Vect3 operator-(const Vect3& vec1, const Vect3& vec2);
Vect3 operator*(const Vect3& vec1, const Vect3& vec2);

enum CameraAngle
{
	XY, TopDown, Side, FPS
};

struct Car
{

	// units in meters
  	Vect3 position, dimensions;
  	Vect3 box1min, box1max;
  	Vect3 box2min, box2max;
  	
  	std::string name;
  	Color color;

  	Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, std::string setName)
    	: position(setPosition),
    	  dimensions(setDimensions),
          box1min ( position + Vect3( -dimensions.x / 2.0, -dimensions.y / 2.0, 0.0) ),
          box1max ( position + Vect3(  dimensions.x / 2.0,  dimensions.y / 2.0, dimensions.z * 2.0 / 3.0) ),
          box2min ( position + Vect3( -dimensions.x / 4.0, -dimensions.y / 2.0, dimensions.z  * 2.0 / 3.0) ),
          box2max ( position + Vect3(  dimensions.x / 4.0,  dimensions.y / 2.0, dimensions.z) ),
    	  color(setColor),
    	  name(setName)
  	{
    }

  	void render(pcl::visualization::PCLVisualizer::Ptr& viewer);

	// collision helper function
	bool inbetween(double point, double center, double range) const
	{
		return (center-range <= point) && (center+range >= point);
	}

	bool checkCollision(const Vect3 &point) const
	{
		return (inbetween(point.x,position.x,dimensions.x/2)&&inbetween(point.y,position.y,dimensions.y/2)&&inbetween(point.z,position.z+dimensions.z/3,dimensions.z/3))||
			   (inbetween(point.x,position.x,dimensions.x/4)&&inbetween(point.y,position.y,dimensions.y/2)&&inbetween(point.z,position.z+dimensions.z*5/6,dimensions.z/6));
	}

	// from chaosTechnician: https://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
    // Helper function for Line/AABB test.  Tests collision on a single dimension
    // Param:    Start of line, Direction/length of line,
    //           Min value of AABB on plane, Max value of AABB on plane
    //           Enter and Exit "timestamps" of intersection (OUT)
    // Return:   True if there is overlap between Line and AABB, False otherwise
    // Note:     Enter and Exit are used for calculations and are only updated in case of intersection
    bool Line_AABB_1d(double start, double dir, double min, double max, double& enter, double& exit) const
    {
        //If the line segment is more of a point, just check if it's within the segment
        if(fabs(dir) < 1.0E-8)
            return (start >= min && start <= max);

        //Find if the lines overlap
        double   ooDir = 1.0 / dir;
        double   t0 = (min - start) * ooDir;
        double   t1 = (max - start) * ooDir;

        //Make sure t0 is the "first" of the intersections
        if(t0 > t1)
            std::swap(t0, t1);

        //Check if intervals are disjoint
        if(t0 > exit || t1 < enter)
        {
            return false;
        }

        //Reduce interval based on intersection
        if(t0 > enter)
            enter = t0;
        if(t1 < exit)
            exit = t1;

        return true;
    }

	// from chaosTechnician: https://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
    // Check collision between a line segment and an AABB
    // Param:    Start point of line segement, End point of line segment,
    //           One corner of AABB, opposite corner of AABB,
    //           Location where line hits the AABB (OUT)
    // Return:   True if a collision occurs, False otherwise
    // Note:     If no collision occurs, OUT param is not reassigned and is not considered useable
    bool Line_AABB(const Vect3& s, const Vect3& dir, const Vect3& min, const Vect3& max, Vect3& hitPoint, double minDist, double &distance) const
    {
        double enter = minDist;
        double exit = distance;

        //Check each dimension of Line/AABB for intersection
        if(!Line_AABB_1d(s.x, dir.x, min.x, max.x, enter, exit))
            return false;
        if(!Line_AABB_1d(s.y, dir.y, min.y, max.y, enter, exit))
            return false;
        if(!Line_AABB_1d(s.z, dir.z, min.z, max.z, enter, exit))
            return false;

        //If there is intersection on all dimensions, report that point
        distance = enter;
        hitPoint = s + distance * dir;

        return true;
    }

    /**
	 * from chaosTechnician: https://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
     * bCheck if ray from origin + a*direction hits the car.
     * @params origin      Starting point of the ray
     *         direction   Direction of the ray - maximal distance * direction must correspond to the actual final distance
     *         hitPoint    If the car is hit, the hitpoint will be set to the closest point
     *         minDist     Minimal distance that needs to be checked
     *         distance    Maximal distance that needs to be checked; if the vehicle is hit, it will contain the distance to the vehicle
     */ 
	bool checkRayCollision(const Vect3 &origin, const Vect3 &direction, Vect3 &hitPoint, double minDist, double &distance) const
    {
        bool hit1 = Line_AABB( origin, direction, box1min, box1max, hitPoint, minDist, distance );
        bool hit2 = Line_AABB( origin, direction, box2min, box2max, hitPoint, minDist, distance );
        
        return ( hit1 || hit2 );
    }
};

void renderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color = Color(1,1,1));
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color = Color(-1,-1,-1));
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color = Color(1,0,0), float opacity=1);
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color = Color(1,0,0), float opacity=1);

#endif
