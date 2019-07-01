/*
 * state_validity_checker_octomap_R2.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: juandhv (Juan David Hernandez Vega, juandhv@eia.udg.edu)
 *      
 *  State checker. Check is a given configuration (R2 state) is collision-free.
 *  The workspace is represented by an octomap.
 */

#include "state_validity_checker_octomap_R2.hpp"

OmFclStateValidityCheckerR2::OmFclStateValidityCheckerR2(const ob::SpaceInformationPtr &si, const double planning_depth, std::vector<double> planning_bounds_x, std::vector<double> planning_bounds_y) :
	ob::StateValidityChecker(si)
{
	GetOctomap::Request req;
	GetOctomap::Response resp;
	std::string serv_name;

	planning_depth_ = planning_depth;
	planning_bounds_x_ = planning_bounds_x;
	planning_bounds_y_ = planning_bounds_y;

	serv_name = "/octomap_binary";
	octree_ = NULL;

	ROS_DEBUG("%s: requesting the map to %s...", ros::this_node::getName().c_str(), node_hand_.resolveName(serv_name).c_str());

	while((node_hand_.ok() && !ros::service::call(serv_name, req, resp)) || resp.map.data.size()==0)
	{
		ROS_WARN("Request to %s failed; trying again...", node_hand_.resolveName(serv_name).c_str());
		usleep(1000000);
	}
	if (node_hand_.ok()){ // skip when CTRL-C
		abs_octree_ = octomap_msgs::msgToMap(resp.map);
		std::cout << std::endl;
		if (abs_octree_){
			octree_ = dynamic_cast<octomap::OcTree*>(abs_octree_);
		}

		octree_->getMetricMin(octree_min_x_, octree_min_y_, octree_min_z_);
		octree_->getMetricMax(octree_max_x_, octree_max_y_, octree_max_z_);
		octree_res_ = octree_->getResolution();
		if (octree_){
			ROS_INFO("%s: Octomap received (%zu nodes, %f m res)", ros::this_node::getName().c_str(), octree_->size(), octree_->getResolution());
		} else{
			ROS_ERROR("Error reading OcTree from stream");
		}
	}
}

bool OmFclStateValidityCheckerR2::isValid(const ob::State *state) const
{
	OcTreeNode* result;
	point3d query;
	bool collision(false);
	double node_occupancy;
	double square_length = 0.5;

	// extract the component of the state and cast it to what we expect
	const ob::RealVectorStateSpace::StateType *pos = state->as<ob::RealVectorStateSpace::StateType>();

	//pos->values[0] = 0.8;
	//pos->values[1] = -0.8;
	for(double xi = pos->values[0]-(square_length/2.0);xi <= pos->values[0]+(square_length/2.0);xi=xi+octree_res_)
		for(double yi = pos->values[1]-(square_length/2.0);yi <= pos->values[1]+(square_length/2.0);yi=yi+octree_res_)
			for(double zi = planning_depth_-(square_length/4.0);zi <= planning_depth_+(square_length/4.0);zi=zi+octree_res_){
				query.x() = xi;
				query.y() = yi;
				query.z() = zi;
				result = octree_->search (query);


				if(result != NULL){
//					collision = false;
//				}
//				else{
					node_occupancy = result->getOccupancy();
					//std::cout << "xi:" << xi << std::endl;
					//std::cout << " node_occupancy: " << node_occupancy << std::endl;
					//std::cout << " octree_res_: " << octree_res_ << std::endl;
					if (node_occupancy > 0.4)
					{
						collision = true;
						break;
					}
				}
		}
	return !collision;
}

OmFclStateValidityCheckerR2::~OmFclStateValidityCheckerR2()
{
    delete octree_;
}
