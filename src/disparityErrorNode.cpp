/*
 * disparityErrorNode.cpp
 *
 *  Created on: May 28, 2014
 *      Author: hannes
 */

#include <ros/ros.h>
#include "disparity_error/DisparityErrorProcessor.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "disparityErrorProcessor");

	ros::NodeHandle nh;

	bool enablePlaneFitting;
	nh.param("/disparityErrorNode/enable_plane_fitting", enablePlaneFitting, false);
	DisparityErrorProcessor proc(nh, enablePlaneFitting);

	ros::spin();

	//return 0;
}


