/*
 * state_validity_checker_octomap_R2.hpp
 *
 *  Created on: Mar 26, 2015
 *      Author: juandhv (Juan David Hernandez Vega, juandhv@eia.udg.edu)
 *      
 *  State checker. Check is a given configuration (R2 state) is collision-free.
 *  The workspace is represented by an octomap.
 */

#ifndef OMPL_CONTRIB_STATE_VALIDITY_CHECKER_OCTOMAP_R2_
#define OMPL_CONTRIB_STATE_VALIDITY_CHECKER_OCTOMAP_R2_

//ROS
#include <ros/ros.h>
//ROS markers rviz
#include <visualization_msgs/Marker.h>
// ROS messages
#include <nav_msgs/Odometry.h>
// ROS tf
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

//Standard libraries
#include <cstdlib>
#include <cmath>
#include <string>

//Octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

//OMPL
#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/debug/Profiler.h>

//Boost
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

//Eigen
#include <Eigen/Dense>

#include <iostream>

//ROS-Octomap interface
using octomap_msgs::GetOctomap;
//Standard namespace
using namespace std;
//Octomap namespace
using namespace octomap;
//OMPL namespaces
namespace ob = ompl::base;
namespace og = ompl::geometric;


//!  OmFclStateValidityCheckerR2 class.
/*!
  Octomap State Validity checker.
  Extension of an abstract class used to implement the state validity checker over an octomap using FCL.
*/
class OmFclStateValidityCheckerR2 : public ob::StateValidityChecker {

public:
	//! OmFclStateValidityCheckerR2 constructor.
	/*!
	 * Besides of initializing the private attributes, it loads the octomap.
	 */
	OmFclStateValidityCheckerR2(const ob::SpaceInformationPtr &si, const double planning_depth, std::vector<double> planning_bounds_x, std::vector<double> planning_bounds_y);

	//! OmFclStateValidityCheckerR2 destructor.
	/*!
	 * Destroy the octomap.
	 */
	~OmFclStateValidityCheckerR2();

	//! State validator.
	/*!
	 * Function that verifies if the given state is valid (i.e. is free of collision) using FCL
	 */
	virtual bool isValid(const ob::State *state) const;
private:
	//ROS
	ros::NodeHandle node_hand_;

	//Octomap
	octomap::AbstractOcTree* abs_octree_;
	octomap::OcTree* octree_;
	double octree_min_x_, octree_min_y_, octree_min_z_;
	double octree_max_x_, octree_max_y_, octree_max_z_;
	std::vector<double> planning_bounds_x_, planning_bounds_y_;

	double planning_depth_, octree_res_;
};

#endif
