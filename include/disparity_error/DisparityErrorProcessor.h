/*
 * DisparityErrorProcessor.h
 *
 *  Created on: May 28, 2014
 *      Author: hannes
 */

#ifndef DISPARITYERRORPROCESSOR_H_
#define DISPARITYERRORPROCESSOR_H_

//ROS
#include <ros/ros.h>

//Messages
#include <shape_msgs/Plane.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

//Services
#include <std_srvs/Empty.h>

//Camera model
#include <image_geometry/stereo_camera_model.h>

//Message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

//Boost
#include <boost/shared_ptr.hpp>

//STL
#include <string>
#include <cmath>
#include <fstream>
#include <iomanip>

class DisparityErrorProcessor {
public:
	typedef message_filters::sync_policies::ApproximateTime<stereo_msgs::DisparityImage, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
	typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

	DisparityErrorProcessor(ros::NodeHandle nh, bool enablePlaneFitting_ = false);
	virtual ~DisparityErrorProcessor();

private:

	//Computes disparity error to the plane if plane fitting is enabled.
	//Only stores the disparity image otherwise.
	void disparityCallback(const stereo_msgs::DisparityImageConstPtr& disparity,
							const sensor_msgs::CameraInfoConstPtr& lCamInfo,
							const sensor_msgs::CameraInfoConstPtr& rCamInfo);

	//Fits a plane to the point cloud using RANSAC and publishes plane parameters.
	void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointCloud2);

	bool toggleFileOutput(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

	//Saves disparity, disparity error, nominal disparity and plane parameters to files.
	bool saveImage() const;

	//Crops the point cloud according to specified cropping parameters.
	bool cropPointCloud(const sensor_msgs::PointCloud2ConstPtr& pointCloud2,
						pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloudOutput) const;

	ros::NodeHandle nh_;

	//Subscribers
	boost::shared_ptr<message_filters::Subscriber<stereo_msgs::DisparityImage> > disparitySub_;
	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > lInfoSub_, rInfoSub_;
	ros::Subscriber pointCloudSub_;

	//Publishers
	ros::Publisher disparityErrorPub_;
	ros::Publisher planePub_;

	//Service Server
	ros::ServiceServer toggleFileOutputSrv_;
	bool enableFileOutput_;
	std::string outputDirectoryPath_;

	//Sync
	boost::shared_ptr<ApproximateSync> disparityApproximateSync_;

	image_geometry::StereoCameraModel model_;
	std::string worldFrameId_, baseFrameId_;
	stereo_msgs::DisparityImage disparityErrorImage_, disparityImage_, nominalDisparityImage_;

	//Plane fitting Parameters
	int xCropMin_, xCropMax_, yCropMin_, yCropMax_;
	double planeFittingThreshold_;
	bool enablePlaneFitting_;

	//Plane parameters
	double planeParameters_[4];
};

#endif /* DISPARITYERRORPROCESSOR_H_ */
