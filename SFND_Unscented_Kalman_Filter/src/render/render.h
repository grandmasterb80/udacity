/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#ifndef RENDER_H
#define RENDER_H
#include <pcl/visualization/pcl_visualizer.h>
#include "box.h"
#include <iostream>
#include <vector>
#include <string>
#include "../ukf.h"

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

struct accuation
{
	long long time_us;
	float acceleration;
	float steering;

	accuation(long long t, float acc, float s)
		: time_us(t), acceleration(acc), steering(s)
	{}
};

struct Car
{

	// units in meters
	Vect3 position, dimensions;
  Vect3 box1min, box1max;
  Vect3 box2min, box2max;

	Eigen::Quaternionf orientation;
	std::string name;
	Color color;
	float velocity;
	float angle;
	float acceleration;
	float steering;
	// distance between front of vehicle and center of gravity
	float Lf;

	UKF ukf;

	//accuation instructions
	std::vector<accuation> instructions;
	int accuateIndex;

	double sinNegTheta;
	double cosNegTheta;

	Car()
		: position(Vect3(0,0,0)), dimensions(Vect3(0,0,0)), color(Color(0,0,0))
	{}
 
	Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, float setVelocity, float setAngle, float setLf, std::string setName)
		: position(setPosition), dimensions(setDimensions),
          box1min ( -dimensions.x / 2.0, -dimensions.y / 2.0, 0.0 ),
          box1max (  dimensions.x / 2.0,  dimensions.y / 2.0, dimensions.z * 2.0 / 3.0 ),
          box2min ( -dimensions.x / 4.0, -dimensions.y / 2.0, dimensions.z  * 2.0 / 3.0 ),
          box2max (  dimensions.x / 4.0,  dimensions.y / 2.0, dimensions.z ),
          color(setColor), velocity(setVelocity), angle(setAngle), Lf(setLf), name(setName)
	{
		orientation = getQuaternion(angle);
		acceleration = 0;
		steering = 0;
		accuateIndex = -1;

		sinNegTheta = sin(-angle);
		cosNegTheta = cos(-angle);
	}

	// angle around z axis
	Eigen::Quaternionf getQuaternion(float theta)
	{
		Eigen::Matrix3f rotation_mat;
  		rotation_mat << 
  		cos(theta), -sin(theta), 0,
    	sin(theta),  cos(theta), 0,
    	0, 			 0, 		 1;
    	
		Eigen::Quaternionf q(rotation_mat);
		return q;
	}

	void render(pcl::visualization::PCLVisualizer::Ptr& viewer)
	{
		// render bottom of car
		viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*1/3), orientation, dimensions.x, dimensions.y, dimensions.z*2/3, name);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name);
		viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*1/3), orientation, dimensions.x, dimensions.y, dimensions.z*2/3, name+"frame");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name+"frame");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name+"frame");
		

		// render top of car
		viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*5/6), orientation, dimensions.x/2, dimensions.y, dimensions.z*1/3, name + "Top");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name + "Top");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name + "Top");
		viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*5/6), orientation, dimensions.x/2, dimensions.y, dimensions.z*1/3, name + "Topframe");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name+"Topframe");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name+"Topframe");
	}

	void setAcceleration(float setAcc)
	{
		acceleration = setAcc;
	}

	void setSteering(float setSteer)
	{
		steering = setSteer;
	}

	void setInstructions(std::vector<accuation> setIn)
	{
		for(accuation a : setIn)
			instructions.push_back(a);
	}

	void setUKF(UKF tracker)
	{
		ukf = tracker;
	}

	void move(float dt, int time_us)
	{

		if(instructions.size() > 0 && accuateIndex < (int)instructions.size()-1)
		{
			if(time_us >= instructions[accuateIndex+1].time_us)
			{
				setAcceleration(instructions[accuateIndex+1].acceleration);
				setSteering(instructions[accuateIndex+1].steering);
				accuateIndex++;
			}
		}

		position.x += velocity * cos(angle) * dt;
		position.y += velocity * sin(angle) * dt;
		angle += velocity*steering*dt/Lf;
		orientation = getQuaternion(angle);
		velocity += acceleration*dt;

		sinNegTheta = sin(-angle);
		cosNegTheta = cos(-angle);
	}

	// collision helper function
	bool inbetween(double point, double center, double range) const
	{
		return (center - range <= point) && (center + range >= point);
	}

	bool checkCollision(const Vect3 &point) const
	{
		// check collision for rotated car
		double xPrime = ((point.x-position.x) * cosNegTheta - (point.y-position.y) * sinNegTheta)+position.x;
		double yPrime = ((point.y-position.y) * cosNegTheta + (point.x-position.x) * sinNegTheta)+position.y;

		return (inbetween(xPrime, position.x, dimensions.x / 2) && inbetween(yPrime, position.y, dimensions.y / 2) && inbetween(point.z, position.z + dimensions.z / 3, dimensions.z / 3)) ||
			(inbetween(xPrime, position.x, dimensions.x / 4) && inbetween(yPrime, position.y, dimensions.y / 2) && inbetween(point.z, position.z + dimensions.z * 5 / 6, dimensions.z / 6));

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
    Vect3 origin2;
    Vect3 direction2;

    origin2.x = cosNegTheta * (origin.x - position.x) - sinNegTheta * (origin.y - position.y);
    origin2.y = sinNegTheta * (origin.x - position.x) + cosNegTheta * (origin.y - position.y);
    direction2.x = cosNegTheta * direction.x - sinNegTheta * direction.x;
    direction2.y = sinNegTheta * direction.x + cosNegTheta * direction.x;

    bool hit1 = Line_AABB( origin2, direction2, box1min, box1max, hitPoint, minDist, distance );
    bool hit2 = Line_AABB( origin2, direction2, box2min, box2max, hitPoint, minDist, distance );
    
    return ( hit1 || hit2 );
  }
};

void renderHighway(double distancePos, pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color = Color(1, 1, 1));
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color = Color(-1, -1, -1));
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color = Color(1, 0, 0), float opacity = 1);
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color = Color(1, 0, 0), float opacity = 1);

#endif
