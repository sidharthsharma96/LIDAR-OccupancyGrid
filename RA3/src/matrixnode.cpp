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

#define m_size (101)
#define scale_factor (5)
// for size = 20 scale_factor = 1;
// factor = 1 means 1/1 meter, factor = 5 means 1/5 meter

// enter padding terms wrt to the center of the vehicle
#define padding (4)
// padding = 1 means , 1/5 meter padding per block. 5 means 5*(1/5) meter padding.
#define lidar_shift_right (1) // positive if right, amount that lidar is dispalced from the center of the vehicle in left right direction
#define lidar_shift_front (3) // positive if froward, amount that lidar is dispalced from the center of the vehicle in front back direction

using namespace std;
int loop_counter = 0;
static int Matrix[m_size][m_size];
nav_msgs::OccupancyGrid map123;
static double myPosition[3];
static double myGoal[3];
static double rotation[3];
static int reached_goal = 0; //if 1 then goal has been reached else 0
int min_distnacet_to_goal = 7; //no of cells
static int goalx = 0;
static int goaly = 0;
int lidarx = (m_size+1)/2;
int lidary = (m_size+1)/2;
int homex = (m_size+1)/2;// + lidar_shift_right;
int homey = (m_size+1)/2;// - lidar_shift_front;
static double sdistance = 0;
static double goalangle = 0;
double precision = 0.01;
static double myHeading = 0;
static double previousx = 0;
static double previousy = 0;
int global_quadrant = 0;
int local_quadrant = 0;
ros::Publisher pub;
//obstacle is 1
//clear is 0

double deg2rad(double deg)
{
	double rad = deg*180/3.14159; 
	return rad; 
}

double ang_wrap(double angle) 
{
	if(angle > 3.14159)
		angle = angle - 2*3.14159;
	if(angle < -3.14159)
		angle = angle + 2*3.14159; 
	return angle;
}

void quadrant_global(double x, double y){
	if (x > 0 && y > 0)
		global_quadrant = 1;
	else if (x < 0 && y > 0)
		global_quadrant = 2;
	else if (x < 0 && y < 0)
		global_quadrant = 3;
	else if (x > 0 && y < 0)
		global_quadrant = 4;
}

void quadrant_local(int x, int y){
	if (x > homex && y > homey)
		local_quadrant = 1;
	else if (x > homex && y < homey)
		local_quadrant = 2;
	else if (x < homex && y < homey)
		local_quadrant = 3;
	else if (x < homex && y > homey)
		local_quadrant = 4;
}


void lidarPosCallback(const geometry_msgs::PoseStampedConstPtr& lidar_pos_msg)
{	  
	myPosition[1] = lidar_pos_msg->pose.position.x;
	myPosition[2] = lidar_pos_msg->pose.position.y;
	rotation[0] = ang_wrap(deg2rad(lidar_pos_msg->pose.orientation.x));
	rotation[1] = ang_wrap(deg2rad(lidar_pos_msg->pose.orientation.y));
	rotation[2] = ang_wrap(deg2rad(lidar_pos_msg->pose.orientation.z));
	quadrant_global(myGoal[1] - myPosition[1],myGoal[2] - myPosition[2]);
	//ROS_INFO("global_quadrant = %d",global_quadrant);
	//double myHeading = ang_wrap(atan2(myPosition[2] - previousy,myPosition[1] - previousx)); //my heading wrt to global coordinates
	//ROS_INFO("My Heading = %f",myHeading);
	sdistance = sqrt((myPosition[1] - myGoal[1])*(myPosition[1] - myGoal[1]) + (myPosition[2] - myGoal[2])*(myPosition[2] - myGoal[2]));
	goalangle = atan2(myGoal[2] - myPosition[2],myGoal[1] - myPosition[1]);
	//ROS_INFO("goalangle = %f",goalangle);
	//previousx = myPosition[1];
	//previousy = myPosition[2];
}

void get_next_goal()
{
	//Checing if final goal has been reached or not, if the goal hasnt been reachged then new next goal coordinates are generated.
	
	if (sdistance >= 5)
	{
		reached_goal = 0;
		for(int a = 0; a < m_size; ++a)
		{
			for (int b = 0; b < m_size; ++b)
			{
				double localangle = atan2(a - homex,b - homey);
				double distnace_local = sqrt((a - homex)*(a - homex) + (b - homey)*(b - homey));
				quadrant_local(a,b);

				if (Matrix[a][b] != 1 && distnace_local >= min_distnacet_to_goal  &&
					 abs(localangle - goalangle) <= precision && local_quadrant == global_quadrant)
				{
					goalx = a;
					goaly = b;
					//ROS_INFO("local_quadrant = %d",local_quadrant);
					//ROS_INFO("%d %d",goalx,goaly);
				}
			}
		}
	}
	//if not getting scan matcher outputs then it is assumed that the goal has been reached
	else{
		reached_goal = 1;
		ROS_INFO("Reached");
	}
}




void ObsCallback(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& msg)
{	
	loop_counter++;
	if(loop_counter%5 == 0)
	{	
		loop_counter = 0;
		std::vector<signed char> v(m_size*m_size);
		pcl::PointXYZRGBL p;
		for(int i = 0; i < msg->width; i++)
		{	
			p = msg->points[i];
			// scaling the coordinates
			p.x = scale_factor*p.x;
			p.y = scale_factor*p.y;
			p.z = scale_factor*p.z;
			
			float rotated[3];
			// x rotation
			rotated[0] = p.x;
			rotated[1] = p.y*cos(rotation[0]) - p.z*sin(rotation[0]);
			rotated[2] = p.y*sin(rotation[0]) + p.z*cos(rotation[0]);
			p.x = rotated[0];
			p.y = rotated[1];
			p.z = rotated[2];
			// y rotation
			rotated[0] = p.z*sin(rotation[1]) + p.x*cos(rotation[1]);
			rotated[1] = p.y;
			rotated[2] = p.z*cos(rotation[1]) - p.x*sin(rotation[1]);
			p.x = rotated[0];
			p.y = rotated[1];
			p.z = rotated[2];
			// z rotation
			rotated[0] = p.x*cos(rotation[2]) - p.y*sin(rotation[2]);
			rotated[1] = p.x*sin(rotation[2]) + p.y*cos(rotation[2]);
			rotated[2] = p.z;
			p.x = rotated[0];
			p.y = rotated[1];
			p.z = rotated[2];
			//translation
			int m = p.x + (m_size+1)/2;
			int l = p.y + (m_size+1)/2;
			
			//filling up the onstacle matrix
			if((l >= padding && m >= padding) && (l < (m_size - padding) && m < (m_size - padding)))
			{
				for (int a = (-padding); a <= padding; ++a)
				{
					for (int b = (-padding); b <= padding; ++b)
					{
						if (l != homey && m != homex)
						{
							v[(l+a)*m_size + (m+b)] = 100;
							Matrix[l+a][m+b] = 1;
						}
						//visulaizing vehicle
						//v[(homex+a)*m_size + lidary] = 80;
						v[(homex+a)*m_size + (homey+b)] = 50;
						//marking vehicle as clear
						Matrix[homex+a][homey+b] = 0;
					}
				}
			}
			
		}

		
		
		
		//Deciding the local goal on the occupancy grid with respect to the velodyne map
		get_next_goal();
		//ROS_INFO("Home(x,y): %d %d",homex,homey);
		//ROS_INFO("Goal(x,y): %d %d",goalx,goaly);
		// for visualising home postion, lidar postion and goal position on rviz
		v[homex*m_size + homey] = 10;
		v[(lidarx)*m_size + (lidary)] = 30;
		v[goalx*m_size + goaly] = 90;

		//creating the text file with goal and home coordinates
		FILE *g = fopen("goal.txt", "w");
		if (g == NULL)
		{
		    printf("Error opening file!\n");
		    exit(1);
		}
		fprintf(g, "%d %d\n", homex,homey);
		fprintf(g, "%d %d", goalx,goaly);
		fclose(g);


		//Creating the text file from occupancy matrix

		FILE *f = fopen("matrix.txt", "w");
		if (f == NULL)
		{
		    printf("Error opening file!\n");
		    exit(1);
		}
		fprintf(f, "%d %d\n", m_size,m_size);

		
		for(int a = 0; a < m_size; a++){
			for (int b = 0; b < m_size; ++b)
			{
				fprintf(f, "%d ", Matrix[a][b]);
			}
			if(a != m_size-1)
				fprintf(f, "\n");
		}
		fclose(f);

		// Launching the Ompl script
		
			//system("/home/usrp-2/shitij/Ompl.sh");
			//int efg = 0;
		if(reached_goal != 0)
		{
			FILE *f = fopen("trajectory_ompl.txt", "w");
			if (f == NULL)
			{
			    printf("Error opening file!\n");
			    exit(1);
			}
			fprintf(f, "0 0\n");
			fclose(f);
		}

		double InputX,InputY;
		ifstream iFile;
		iFile.open("trajectory_ompl.txt");
		while((iFile >> InputX) && (iFile >> InputY))
		  {
		    v[int(InputX)*m_size + int(InputY)] = 70;
		    //ROS_INFO("%f %f",InputX,InputY);
		  }
		iFile.close();

		map123.data = v;
		pub.publish(map123);
	}
}


int main(int argc, char** argv)
{
ros::init(argc, argv, "matrix_node");
ros::NodeHandle n;

map123.info.resolution = 1;         
map123.info.width      = m_size;
map123.info.height     = m_size;
	

ifstream iFile2;
double X_goal_final,Y_goal_final;
iFile2.open("trajectoryLidar.txt");
if ((iFile2 >> X_goal_final) && (iFile2 >> Y_goal_final))
{
  	myGoal[1] = X_goal_final;
	myGoal[2] = Y_goal_final;
}
iFile2.close();

pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1);
ros::Subscriber Obs = n.subscribe("velodyne_obstacles",1,ObsCallback);
ros::Subscriber lidar_pos = n.subscribe<geometry_msgs::PoseStamped>("/trajectory", 1, lidarPosCallback);

while (ros::ok()){
	ros::spin();
}

return 0;
}
