#ifndef LIDAR_H
#define LIDAR_H
#include "../render/render.h"
#include <ctime>
#include <chrono>

const double pi = 3.1415;

struct Ray
{
	
	Vect3 origin;
	double resolution;
	Vect3 direction;
	Vect3 castPosition;
	double castDistance;

	// parameters:
	// setOrigin: the starting position from where the ray is cast
	// horizontalAngle: the angle of direction the ray travels on the xy plane
	// verticalAngle: the angle of direction between xy plane and ray 
	// 				  for example 0 radians is along xy plane and pi/2 radians is stright up
	// resolution: the magnitude of the ray's step, used for ray casting, the smaller the more accurate but the more expensive

	Ray(Vect3 setOrigin, double horizontalAngle, double verticalAngle, double setResolution)
		: origin( setOrigin ),
		  resolution( setResolution ),
		  direction( cos( verticalAngle ) * cos( horizontalAngle ), cos( verticalAngle ) * sin( horizontalAngle ), sin( verticalAngle ) ),
		  castPosition( origin ),
		  castDistance( 0 )
	{
        direction *= resolution;
    }

	void rayCast(const std::vector<Car>& cars, double minDistance, double maxDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double slopeAngle, double sderr)
	{
		// reset ray
		castPosition = origin;
		castDistance = 0;

		bool collision = false;

		while(!collision && castDistance < maxDistance)
		{

			castPosition = castPosition + direction;
			castDistance += resolution;

			// check if there is any collisions with ground slope
			collision = (castPosition.z <= castPosition.x * tan(slopeAngle));

			// check if there is any collisions with cars
			if(!collision && castDistance < maxDistance)
			{
				for(Car car : cars)
				{
					collision |= car.checkCollision(castPosition);
					if(collision)
						break;
				}
			}
		}

		if((castDistance >= minDistance)&&(castDistance<=maxDistance))
		{
			// add noise based on standard deviation error
			double rx = ((double) rand() / (RAND_MAX));
			double ry = ((double) rand() / (RAND_MAX));
			double rz = ((double) rand() / (RAND_MAX));
			cloud->points.push_back(pcl::PointXYZ(castPosition.x+rx*sderr, castPosition.y+ry*sderr, castPosition.z+rz*sderr));
		}
			
	}

};

struct Lidar
{

	std::vector<Ray> rays;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::vector<Car> cars;
	Vect3 position;
	double groundSlope;
    Vect3 groundNormal;
	double minDistance;
	double maxDistance;
	double resolution;
	double sderr;

    /// Value that can be used to compute #points in specific applications (e.g. expected #points in a cluster of an object).
    /// think of #points / square meter @ 1m distance
    float density;

	Lidar(std::vector<Car> setCars, double setGroundSlope)
		: cloud(new pcl::PointCloud<pcl::PointXYZ>()), position(0,0,2.6), groundNormal( sin ( setGroundSlope ), 0.0, cos( setGroundSlope ) )
	{
		// TODO:: set minDistance to 5 to remove points from roof of ego car
		minDistance = 2;
		maxDistance = 120;
		resolution = 0.2;
		// TODO:: set sderr to 0.2 to get more interesting pcd files
		sderr = 0.2;
		cars = setCars;
		groundSlope = setGroundSlope;

		// TODO:: increase number of layers to 8 to get higher resolution pcd
		int numLayers = 64; //8;
		// the steepest vertical angle
		double steepestAngle = -24.9*pi/180; // 30.0*(-pi/180);
		double angleRange = 26.9*pi/180; //26.0*(pi/180);
		// TODO:: set to pi/64 to get higher resolution pcd
		double horizontalAngleInc = 0.08*pi/180;  // pi/64

		double angleIncrement = angleRange/numLayers;

		for(double angleVertical = steepestAngle; angleVertical < steepestAngle+angleRange; angleVertical+=angleIncrement)
		{
			for(double angle = 0; angle <= 2*pi; angle+=horizontalAngleInc)
			{
				Ray ray(position,angle,angleVertical,resolution);
				rays.push_back(ray);
			}
		}

        density = rays.size() / ( 2.0 * pi ) / fabs( sin( steepestAngle ) - sin( steepestAngle + angleRange ) );
        cout << "Lidar has about a density of " << density << " points per square meter at 1m distance" << endl;
	}

	~Lidar()
	{
		// pcl uses boost smart pointers for cloud pointer so we don't have to worry about manually freeing the memory
	}

	inline float PointDensity() { return density; }

	pcl::PointCloud<pcl::PointXYZ>::Ptr scan()
	{
		cloud->points.clear();
		auto startTime = std::chrono::steady_clock::now();

// #pragma omp declare reduction (merge : pcl::PointCloud<pcl::PointXYZ>::Ptr : omp_out->insert(omp_out->end(), omp_in->begin(), omp_in->end()))
// #pragma omp parallel for reduction(merge: cloud)
		//for(Ray ray : rays)
		//	ray.rayCast(cars, minDistance, maxDistance, cloud, groundSlope, sderr);

#if defined(_OPENMP)
        {        
            int max_threads = omp_get_max_threads();
            int num_rays = rays.size();
            #pragma omp parallel shared(max_threads, num_rays, cars, minDistance, maxDistance, cloud, groundSlope, sderr)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr sub_cloud ( new pcl::PointCloud<pcl::PointXYZ>() );
                #pragma omp for
                for(int i = 0; i < max_threads; ++i)
                {
                    int firstElement = num_rays * i / max_threads;
                    int lastElement = num_rays * ( i + 1 ) / max_threads - 1;
                    for(std::vector<Ray>::iterator ray = rays.begin() + firstElement; ray != rays.begin() + lastElement; ray++)
                        ray->rayCast(cars, minDistance, maxDistance, sub_cloud, groundSlope, sderr);
                    #pragma omp critical
                    cloud->insert( cloud->end(), sub_cloud->begin(), sub_cloud->end() );
                }
            }
        }
#else
		for(Ray ray : rays)
			ray.rayCast(cars, minDistance, maxDistance, cloud, groundSlope, sderr);
#endif

        cout << "#rays=" << rays.size() << "    #points=" << cloud->size() << endl;
		auto endTime = std::chrono::steady_clock::now();
		auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
		cout << "ray casting took " << elapsedTime.count() << " milliseconds" << endl;
		cloud->width = cloud->points.size();
		cloud->height = 1; // one dimensional unorganized point cloud dataset
		return cloud;
	}

};

#endif
