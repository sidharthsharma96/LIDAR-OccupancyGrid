#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/pcl_base.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/crop_box.h"
#include "math.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
#include <string>

//subscribe to a couple of topics and publish to another couple of topics here 

typedef pcl::PointXYZI VPoint ;
typedef pcl::PointCloud<VPoint> VPointCloud ;

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))
int count = 0;
void callback( const VPointCloud::ConstPtr& msg)
{

	ROS_INFO("Cloud:width = %d , height=%d",msg->width,msg->height) ;
	float m_per_cell_ =  0.1;
	int grid_dim_ = 101;
	float height_diff_threshold_ = 0.01;
	float min[grid_dim_][grid_dim_];
	float max[grid_dim_][grid_dim_];
	bool init[grid_dim_][grid_dim_];
	float ob[grid_dim_][grid_dim_];
  	memset(&init, 0, grid_dim_*grid_dim_);

	BOOST_FOREACH(const VPoint& point,msg->points)
	{
		int x = ((grid_dim_/2) + point.x/m_per_cell_);
    	int y = ((grid_dim_/2) + point.y/m_per_cell_);
    	if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) 
    	{
      		if (!init[x][y]) 
      		{
        		min[x][y] = point.z;
        		max[x][y] = point.z;
        		init[x][y] = true;
      		} 
      		else 
      		{
        		min[x][y] = MIN(min[x][y], point.z);
        		max[x][y] = MAX(max[x][y], point.z);
      		}
    	}

	}

	BOOST_FOREACH(const VPoint& point,msg->points)
	{
		
		int x = ((grid_dim_/2) + point.x/m_per_cell_);
    	int y = ((grid_dim_/2) + point.y/m_per_cell_);
    	if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) 
    	{
      		if ((max[x][y] - min[x][y] > height_diff_threshold_) ) 
      		{   
        		ob[x][y] = 1;
      		}
      		else
      		{
      			ob[x][y] = 0;
      		}
    	}
	}
	std::ofstream myfile;
	std::string s = boost::lexical_cast<std::string>(count);
	std::string s1 = "/home/sidharth/catkin_ws/src/RA3/scripts/matrix" + s +".txt";
  	myfile.open (s1.c_str());
  	count++;
	for(int i=0;i<100;i++)
		{
			for(int j=0;j<100;j++)
			{
				myfile << ob[i][j] <<" ";
			}
			myfile <<"\n";
		}
	std::cout<<count<<"\n";
 	myfile.close();

}


int main(int argc , char ** argv){

	ros::init(argc , argv , "velodyne_example_sub") ;
	ros::NodeHandle nh ;

	ros::Subscriber sub = nh.subscribe<VPointCloud>("/velodyne_points",1,callback);

	ros::spin() ;


}