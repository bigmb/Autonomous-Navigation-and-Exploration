/*
 * offline_planner_R2_main.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: juandhv (Juan David Hernandez Vega, juandhv@eia.udg.edu)
 *      
 *  Offline path planning (without differential constraints) using OMPL.
 *  Octomaps are used to represent workspace and to validate if configurations are collision-free.
 */

// *** OMPL ***
//#include <ompl/control/SpaceInformation.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// *** ROS ***
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <autonomous_explore_map_plan/FindPathToGoal.h>
#include <geometry_msgs/Pose2D.h>

// *** IO ***
#include <iostream>
#include <string>

// *** state validity checker, collision checking using Octomaps***
#include "state_validity_checker_octomap_R2.hpp"

namespace ob = ompl::base;

//!  OfflinePlannerR2 class.
/*!
 * Offline Planner.
 * Setup a sampling-based planner for computing a collision-free path.
 * C-Space: R2
 * Workspace is represented with Octomaps
*/
class OfflinePlannerR2
{
    public:
		//! Constructor
		OfflinePlannerR2();
		//! Planner setup
		void planWithSimpleSetup(std::vector<double> start_state, std::vector<double> goal_state, autonomous_explore_map_plan::FindPathToGoal::Response& response);
		//! Procedure to visualize the resulting path
		void visualizeRRT(og::PathGeometric geopath);
		//! Procedure to create a mission file using the resulting path
		void createMissionFile(og::PathGeometric geopath);
		//! Callback for getting current vehicle position
		void odomCallback(const nav_msgs::OdometryPtr &odom_msg);
		//! Callback for setting the query goal to specified values
		bool findPathToGoal(autonomous_explore_map_plan::FindPathToGoal::Request& request, autonomous_explore_map_plan::FindPathToGoal::Response& response);
    private:
		// *** ROS ***
		ros::NodeHandle node_handler_;
		ros::Publisher online_traj_pub_;
		ros::Subscriber odom_sub_;
		ros::ServiceServer service_;
		// *** OMPL ***
		og::SimpleSetupPtr simple_setup_;
		// *** Planner parameters ***
		std::vector<double> planning_bounds_x_, planning_bounds_y_, current_position_;
		double planning_depth_, solving_time_;
		std::string planner_name_;
};

//!  Constructor.
/*!
 * Load planner parameters from configuration file.
 * Publishers to visualize the resulting path.
*/
OfflinePlannerR2::OfflinePlannerR2()
{
	//=======================================================================
	// Subscribers.
	//=======================================================================
    //Navigation data
    odom_sub_ = node_handler_.subscribe("/odom", 1, &OfflinePlannerR2::odomCallback, this);

	//=======================================================================
	// Publishers (to visualize the resulting path).
	//=======================================================================
	online_traj_pub_ = node_handler_.advertise<visualization_msgs::Marker> ("/controller_turtlebot/solution_path", 1, true);

	//=======================================================================
	// Missions are defined at a constant depth, thus C-Space dimension is R2.
	//=======================================================================
	planning_bounds_x_.resize(2);
	planning_bounds_y_.resize(2);
	current_position_.resize(2);

	//=======================================================================
	// Get parameters from configuration file.
	//=======================================================================
	node_handler_.getParam("planning_bounds_x", planning_bounds_x_);
	node_handler_.getParam("planning_bounds_y", planning_bounds_y_);
	//node_handler_.getParam("solving_time", solving_time_);
	//node_handler_.getParam("planner_name", planner_name_);
	planning_depth_ = 0.3;
	solving_time_ = 50;
	planner_name_ = "RRT";
	//node_handler_.planner_name_ = "RRTstar";
	//node_handler_.solving_time_ = 20;
	//=======================================================================
	// Service
	//=======================================================================
	service_ = node_handler_.advertiseService("/turtlebot_return/find_path_to_goal", &OfflinePlannerR2::findPathToGoal, this);
}

//!  Planner setup.
/*!
 * Setup a sampling-based planner using OMPL.
*/
void OfflinePlannerR2::planWithSimpleSetup(std::vector<double> start_state, std::vector<double> goal_state, autonomous_explore_map_plan::FindPathToGoal::Response& response)
{
	//=======================================================================
	// Instantiate the state space (R2, X-Y). Navigating at a constant depth.
	//=======================================================================
	ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

	// Set the bounds for the state space
	ob::RealVectorBounds bounds(2);

	bounds.setLow(0, planning_bounds_x_[0]);
	bounds.setHigh(0, planning_bounds_x_[1]);
	bounds.setLow(1, planning_bounds_y_[0]);
	bounds.setHigh(1, planning_bounds_y_[1]);

	space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

	//=======================================================================
	// Define a simple setup class, which contains the planning problem.
	//=======================================================================
	simple_setup_ = og::SimpleSetupPtr( new og::SimpleSetup(space) );
	ob::SpaceInformationPtr si = simple_setup_->getSpaceInformation();
	
	//=======================================================================
	// Create a planner. OMPL is specialized in sampling-based methods
	//=======================================================================
	// (RRT, RRTstar, RRTConnect), (EST), (PRM, PRMstar)
	ob::PlannerPtr planner;
	if(planner_name_.compare("RRT")==0)
		planner = ob::PlannerPtr(new og::RRT(si));
	else if(planner_name_.compare("RRTstar")==0)
		planner = ob::PlannerPtr(new og::RRTstar(si));
	else if(planner_name_.compare("RRTConnect")==0)
		planner = ob::PlannerPtr(new og::RRTConnect(si));
	else if(planner_name_.compare("EST")==0)
		planner = ob::PlannerPtr(new og::EST(si));
	else if(planner_name_.compare("PRM")==0)
		planner = ob::PlannerPtr(new og::PRM(si));
	else if(planner_name_.compare("PRMstar")==0)
		planner = ob::PlannerPtr(new og::PRMstar(si));
	else
		planner = ob::PlannerPtr(new og::RRT(si));

	//planner->as<og::RRTstar>()->setRange(0.2);
	//=======================================================================
	// Set the setup planner
	//=======================================================================
	simple_setup_->setPlanner(planner);

	//=======================================================================
	// Set state validity checking for this space.
	//=======================================================================
	OmFclStateValidityCheckerR2 * om_stat_val_check = new OmFclStateValidityCheckerR2(si, planning_depth_, planning_bounds_x_, planning_bounds_y_);
	simple_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr(om_stat_val_check));

	//=======================================================================
	// Create a start and goal states
	//=======================================================================
	ob::ScopedState<> start(space);

	start[0] = double(start_state[0]);
	start[1] = double(start_state[1]);

	// create a goal state
	ob::ScopedState<> goal(space);

	goal[0] = double(goal_state[0]);
	goal[1] = double(goal_state[1]);

	//=======================================================================
	// Set the start and goal states
	//=======================================================================
	simple_setup_->setStartAndGoalStates(start, goal);

	//=======================================================================
	// Perform setup steps for the planner
	//=======================================================================
	simple_setup_->setup();

	//=======================================================================
	// Print planner information
	//=======================================================================
	//planner->printProperties(std::cout);// print planner properties
	//si->printSettings(std::cout);// print the settings for this space

	//=======================================================================
	// Attempt to solve the query
	//=======================================================================
	ob::PlannerStatus solved = simple_setup_->solve( solving_time_ );

	if (solved && simple_setup_->haveExactSolutionPath())
	{
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		//		simple_setup_->simplifySolution();
		og::PathGeometric path = simple_setup_->getSolutionPath();
		//path.interpolate(int(path.length()/1.0));
		visualizeRRT(path);

		std::vector< ob::State * > states = path.getStates();
		const ob::RealVectorStateSpace::StateType *state;
		for (uint32_t i = 0; i < path.getStateCount(); ++i)
		{
			geometry_msgs::Pose2D pose;
			// extract the component of the state and cast it to what we expect
			state = states[i]->as<ob::RealVectorStateSpace::StateType>();

			pose.x = state->values[0];
			pose.y = state->values[1];

			response.poses.push_back(pose);
		}


		ROS_INFO("%s: path has been found with simple_setup", ros::this_node::getName().c_str());
	}
	else
		ROS_INFO("%s: path has not been found", ros::this_node::getName().c_str());
}

//!  Resulting path visualization.
/*!
 * Visualize resulting path.
*/
void OfflinePlannerR2::visualizeRRT(og::PathGeometric geopath)
{
	// %Tag(MARKER_INIT)%
	visualization_msgs::Marker q_init_goal, visual_rrt, result_path, visual_result_path;
	visual_result_path.header.frame_id = result_path.header.frame_id = q_init_goal.header.frame_id = visual_rrt.header.frame_id =  "/odom";
	visual_result_path.header.stamp = result_path.header.stamp = q_init_goal.header.stamp = visual_rrt.header.stamp = ros::Time::now();
	q_init_goal.ns = "online_planner_points";
	visual_rrt.ns = "online_planner_rrt";
	result_path.ns = "online_planner_result";
	visual_result_path.ns = "online_planner_result_path";
	visual_result_path.action = result_path.action = q_init_goal.action = visual_rrt.action = visualization_msgs::Marker::ADD;

	visual_result_path.pose.orientation.w = result_path.pose.orientation.w = q_init_goal.pose.orientation.w = visual_rrt.pose.orientation.w = 1.0;
	// %EndTag(MARKER_INIT)%

	// %Tag(ID)%
	q_init_goal.id = 0;
	visual_rrt.id = 1;
	result_path.id = 2;
	visual_result_path.id = 3;
	// %EndTag(ID)%

	// %Tag(TYPE)%
	result_path.type = q_init_goal.type = visualization_msgs::Marker::POINTS;
	visual_rrt.type = visual_result_path.type = visualization_msgs::Marker::LINE_LIST;
	// %EndTag(TYPE)%

	// %Tag(SCALE)%
	// POINTS markers use x and y scale for width/height respectively
	result_path.scale.x = q_init_goal.scale.x = 0.5;
	result_path.scale.y = q_init_goal.scale.y = 0.5;
	result_path.scale.z = q_init_goal.scale.z = 0.5;

	// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
	visual_rrt.scale.x = 0.01;
	visual_result_path.scale.x = 0.02;
	// %EndTag(SCALE)%

	// %Tag(COLOR)%
	// Points are green
	visual_result_path.color.g = 1.0;
	result_path.color.g = q_init_goal.color.g = 1.0;
	visual_result_path.color.a = result_path.color.a = q_init_goal.color.a = 1.0;

	// Line strip is blue
	visual_rrt.color.b = 1.0;
	visual_rrt.color.a = 1.0;


	ob::PlannerData planner_data(simple_setup_->getSpaceInformation());
	simple_setup_->getPlannerData(planner_data);
	std::vector< unsigned int > edgeList;
	int num_parents;
	const ob::SE2StateSpace::StateType *state_se2;
	const ob::RealVectorStateSpace::StateType *state_r2;

	const ob::RealVectorStateSpace::StateType *state;
	geometry_msgs::Point p;

	for (unsigned int i = 1 ; i < planner_data.numVertices() ; ++i)
	{
		if (planner_data.getVertex(i).getState() && planner_data.getIncomingEdges(i,edgeList) > 0)
		{
			state_r2 = planner_data.getVertex(i).getState()->as<ob::RealVectorStateSpace::StateType>();
			p.x = state_r2->values[0];
			p.y = state_r2->values[1];
			p.z = planning_depth_;
			visual_rrt.points.push_back(p);

			state_r2 = planner_data.getVertex(edgeList[0]).getState()->as<ob::RealVectorStateSpace::StateType>();
			p.x = state_r2->values[0];
			p.y = state_r2->values[1];
			p.z = planning_depth_;
			visual_rrt.points.push_back(p);
		}
	}
	std::vector< ob::State * > states = geopath.getStates();

	for (uint32_t i = 0; i < geopath.getStateCount(); ++i)
	{
		// extract the component of the state and cast it to what we expect
		state = states[i]->as<ob::RealVectorStateSpace::StateType>();

		p.x = state->values[0];
		p.y = state->values[1];
		p.z = planning_depth_;//pos->values[2];

		result_path.points.push_back(p);

		if(i>0)
		{
			visual_result_path.points.push_back(p);
			state = states[i-1]->as<ob::RealVectorStateSpace::StateType>();

			p.x = state->values[0];
			p.y = state->values[1];
			p.z = planning_depth_;
			visual_result_path.points.push_back(p);
		}
	}

	//online_traj_pub_.publish(q_init_goal);
	online_traj_pub_.publish(visual_rrt);
	online_traj_pub_.publish(visual_result_path);
	//online_traj_pub_.publish(result_path);
}


//! odomCallback.
/*!
 * Callback for getting updated vehicle position
*/
void OfflinePlannerR2::odomCallback(const nav_msgs::OdometryPtr &odom_msg)
{
	current_position_[0] = odom_msg->pose.pose.position.x;
	current_position_[1] = odom_msg->pose.pose.position.y;
}

//! setGoalCallback.
/*!
 * Service callback that sets the value of the query goal to the value of current position
*/
bool OfflinePlannerR2::findPathToGoal(autonomous_explore_map_plan::FindPathToGoal::Request& request, autonomous_explore_map_plan::FindPathToGoal::Response& response){
	ROS_INFO("%s: finding a path from current position to a given goal", ros::this_node::getName().c_str());
	vector<double> goal_state;
	goal_state.resize(2);
	goal_state[0] = request.goal_state_x;
	goal_state[1] = request.goal_state_y;

	planWithSimpleSetup(current_position_, goal_state, response);
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "offline_planner_with_services_R2");
	ros::NodeHandle node_handler;
	ROS_INFO("%s: offline planner (C++)", ros::this_node::getName().c_str());
	ROS_INFO("%s: using OMPL version %s", ros::this_node::getName().c_str(), OMPL_VERSION);
	ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

	OfflinePlannerR2 offline_planner;

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
