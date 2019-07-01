// ROS
#include <ros/ros.h>

// ROS LaserScan tools
#include <laser_geometry/laser_geometry.h>

// ROS messages
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>

// ROS services
#include <std_srvs/Empty.h>

// ROS tf
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

// Octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
//#include <octomap_msgs/OctomapBinary.h>
#include <octomap_msgs/GetOctomap.h>
typedef octomap_msgs::GetOctomap OctomapSrv;
//#include <laser_octomap/BoundingBoxQuery.h>
//typedef laser_octomap::BoundingBoxQuery BBXSrv;
#include <octomap_ros/conversions.h>

#include <signal.h>


void stopNode(int sig)
{
	ros::shutdown();
	exit(0);
}

/// CLASS DEFINITION ===========================================================
class LaserScanToPointCloud
{
    public:
        // Constructor and destructor
        LaserScanToPointCloud();
//        virtual ~LaserScanToPointCloud();

        // Callbacks
        void laserScanCallback(const sensor_msgs::LaserScanConstPtr& scan);

    private:

        // ROS
        ros::NodeHandle node_hand_;
        ros::Subscriber laser_scan_sub_;
        ros::Publisher pc_pub_;

        //TF
        tf::TransformListener listener_;

        //PC
        laser_geometry::LaserProjection projector_;
};

/// Constructor and destructor =================================================
LaserScanToPointCloud::LaserScanToPointCloud(){
    //=======================================================================
    // Subscribers
    //=======================================================================
    //Mission Flag (feedback)
	laser_scan_sub_ = node_hand_.subscribe("/scan", 2, &LaserScanToPointCloud::laserScanCallback, this);

	//=======================================================================
	// Publishers
	//=======================================================================
	pc_pub_ = node_hand_.advertise<sensor_msgs::PointCloud2>("pc_from_scan", 2, true);
//    // Waiting for mission flag
//    ros::Rate loop_rate(10);
//    if(!mission_flag_available_)
//    	ROS_WARN("Waiting for mission_flag");
//    while (ros::ok() && !mission_flag_available_)
//    {
//    	ros::spinOnce();
//    	loop_rate.sleep();
//    }
//    //Navigation data (feedback)
//    odomSub_ = node_hand_.subscribe("/pose_ekf_slam/odometry", 1, &LaserScanToPointCloud::odomCallback, this);
//    nav_sts_available_ = false;
//    while (ros::ok() && !nav_sts_available_)
//    {
//    	ros::spinOnce();
//    	loop_rate.sleep();
//    }
}

void LaserScanToPointCloud::laserScanCallback(const sensor_msgs::LaserScanConstPtr &scan){
	if(!listener_.waitForTransform(
			scan->header.frame_id,
			"/odom",
			scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
			ros::Duration(1.0))){
		return;
	}

	sensor_msgs::PointCloud2 cloud;
	projector_.transformLaserScanToPointCloud("/base_link",*scan,
			cloud,listener_);

	  // Do something with cloud.
	pc_pub_.publish(cloud);
}

/// MAIN NODE FUNCTION =========================================================
int main(int argc, char** argv){

	//=======================================================================
	// Override SIGINT handler
	//=======================================================================
	signal(SIGINT, stopNode);

	// Init ROS node
	ros::init(argc, argv, "laserscan_to_pointcloud");
	ros::NodeHandle private_nh("~");

	// Constructor
	LaserScanToPointCloud laserscan_to_pc;

	// Spin
	ros::spin();

	// Exit main function without errors
	return 0;
}
