/*
 * DisparityErrorProcessor.cpp
 *
 *  Created on: May 28, 2014
 *      Author: hannes
 */

#include "disparity_error/DisparityErrorProcessor.h"

DisparityErrorProcessor::DisparityErrorProcessor(ros::NodeHandle nh, bool enablePlaneFitting):
    nh_(nh),  enableFileOutput_(false), worldFrameId_("/world"), baseFrameId_("/cam0"),
    enablePlaneFitting_(enablePlaneFitting)
{
	ROS_INFO("Initializing DisparityErrorProcessor...");

  //Initialize subscribers
	disparitySub_.reset(new message_filters::Subscriber<stereo_msgs::DisparityImage> (nh_, "/disparity", 50));
	lInfoSub_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo> (nh_, "/cam0/camera_info", 50));
	rInfoSub_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo> (nh_, "/cam1/camera_info", 50));

  //Initialize plane fitting related functionality if necessary.
  if(enablePlaneFitting_)
  {
    ROS_INFO("Plane fitting is enabled.");
    pointCloudSub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points2", 1000, &DisparityErrorProcessor::pointCloudCallback, this);
    planePub_ = nh_.advertise<shape_msgs::Plane> ("fitted_plane", 1000);
    disparityErrorPub_ = nh_.advertise<stereo_msgs::DisparityImage> ("disparity_error", 1000);

    //Get cropping parameters from ROS
    nh_.param("/disparityErrorNode/x_crop_min", xCropMin_, 0);
    nh_.param("/disparityErrorNode/x_crop_max", xCropMax_, 0);
    nh_.param("/disparityErrorNode/y_crop_min", yCropMin_, 0);
    nh_.param("/disparityErrorNode/y_crop_max", yCropMax_, 0);
    nh_.param("/disparityErrorNode/plane_fitting_threshold", planeFittingThreshold_, 0.01);

    //Initialize plane parameters to valid values
    for(int i = 0; i < 4; i++)
      planeParameters_[i] = 1;
  }

	//Initialize synchronized disparity callback
	disparityApproximateSync_.reset(new ApproximateSync(ApproximatePolicy(5), *disparitySub_, *lInfoSub_, *rInfoSub_));
	disparityApproximateSync_->registerCallback(boost::bind(&DisparityErrorProcessor::disparityCallback, this, _1, _2, _3));

	//Initialize image service server
	toggleFileOutputSrv_ = nh_.advertiseService("/disparityErrorNode/toggleFileOutput", &DisparityErrorProcessor::toggleFileOutput, this);


}

DisparityErrorProcessor::~DisparityErrorProcessor() {}

void DisparityErrorProcessor::disparityCallback(const stereo_msgs::DisparityImageConstPtr& disparity,
		const sensor_msgs::CameraInfoConstPtr& lCamInfo,
		const sensor_msgs::CameraInfoConstPtr& rCamInfo)
{
	ros::WallTime startTime = ros::WallTime::now();

	// Copy camera_info messages and modify if necessary
	sensor_msgs::CameraInfo l_info_msg_copy(*lCamInfo);
	sensor_msgs::CameraInfo r_info_msg_copy(*rCamInfo);

	if (-rCamInfo->P[3]/lCamInfo->K[0] <= 0.0)
	{
		r_info_msg_copy.P[3] = -l_info_msg_copy.P[3];
		l_info_msg_copy.P[3] = 0.0;
	}
	l_info_msg_copy.header.frame_id = baseFrameId_;
	r_info_msg_copy.header.frame_id = l_info_msg_copy.header.frame_id;

	// Update the camera model
	model_.fromCameraInfo(l_info_msg_copy, r_info_msg_copy);

	if(enablePlaneFitting_)
	{
	  //Subtract nominal disparity from measured disparity for every pixel in the image
	  disparityImage_ = *disparity;
	  disparityErrorImage_ = *disparity;
	  nominalDisparityImage_ = *disparity;
	  float* data_ptr = (float*)disparityErrorImage_.image.data.data();
	  float* nom_ptr = (float*)nominalDisparityImage_.image.data.data();

	  float x1 = model_.right().cx() - model_.left().cx();
	  float x2 = -model_.baseline()/planeParameters_[3];
	  float x3 = planeParameters_[2]*model_.right().fx();

	  for(unsigned int v = 0; v < disparityErrorImage_.image.height; v++)
	  {
	    for(unsigned int u = 0; u < disparityErrorImage_.image.width; u++)
	    {
	      nom_ptr[nominalDisparityImage_.image.width*v + u] = x1 + x2*(planeParameters_[0]*(u - model_.right().cx()) + planeParameters_[1]*(v - model_.right().cy()) + x3);
	      if(data_ptr[disparityErrorImage_.image.width*v + u] != -1)
	        data_ptr[disparityErrorImage_.image.width*v + u] = pow(data_ptr[disparityErrorImage_.image.width*v + u] - nom_ptr[nominalDisparityImage_.image.width*v + u], 2);
	    }
	  }

	  disparityErrorPub_.publish(disparityErrorImage_);
	}

	if(enableFileOutput_)
		saveImage();
}

void DisparityErrorProcessor::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointCloud2)
{
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

	if(!cropPointCloud(pointCloud2, croppedCloud))
		return;

	//Fit the plane with RANSAC
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr plane_model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (croppedCloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(plane_model);

	ransac.setDistanceThreshold(planeFittingThreshold_);
	ransac.computeModel();

	//Extract model coefficients
	Eigen::VectorXf model_coefficients;
	ransac.getModelCoefficients(model_coefficients);

	//Publish the fitted plane
	shape_msgs::Plane fitted_plane;
	for(int i = 0; i < 4; i++)
	{
		fitted_plane.coef[i] = model_coefficients(i);
		planeParameters_[i] = model_coefficients(i);
	}

	planePub_.publish(fitted_plane);
}

bool DisparityErrorProcessor::toggleFileOutput(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
	enableFileOutput_ = !enableFileOutput_;
	nh_.param("/disparityErrorNode/outputDirectoryPath", outputDirectoryPath_, std::string("/home/"));
	if(outputDirectoryPath_[outputDirectoryPath_.length() -1] != '/')
		outputDirectoryPath_.append("/");
	if(enableFileOutput_)
		ROS_INFO("Enabled file output to directory %s", outputDirectoryPath_.c_str());
	else
		ROS_INFO("Disabled file output.");

	//Save parameter info
	std::stringstream filePath;
	filePath << outputDirectoryPath_ << "parameter_info.txt";
	std::ofstream fileStream(filePath.str().c_str());

	int disparityRange;
	nh_.param("/stereo_image_proc/disparity_range", disparityRange, 0);

	fileStream << "Max. Disparity: " << disparityRange << std::endl;
	fileStream << "Cropping parameters" << std::endl;
	fileStream << "xMin: " << xCropMin_ << "\nxMax: " << xCropMax_ << "\nyMin: " << yCropMin_ << "\nyMax: " << yCropMax_ << std::endl;

	return true;
}

bool DisparityErrorProcessor::saveImage() const
{
	std::stringstream filePath;
	std::ofstream fileStream;
	float* data_ptr;

	//Save disparity image
	filePath.str(std::string());
	filePath << outputDirectoryPath_ << "disparity/" << disparityErrorImage_.header.stamp << "_disp.csv";

	fileStream.close();
	fileStream.open(filePath.str().c_str());
	if(!fileStream.is_open())
	{
		ROS_ERROR_STREAM("Could not open output file.");
		return false;
	}

	data_ptr = (float*)disparityImage_.image.data.data();
	for(unsigned int v = 0; v < disparityImage_.image.height; v++)
	{
		for(unsigned int u = 0; u < disparityImage_.image.width; u++)
		{
			fileStream << std::scientific << std::setprecision(6) << data_ptr[disparityImage_.image.width*v + u] << "\t";
		}
		fileStream << std::endl;
	}

	if(enablePlaneFitting_)
	{
	  //Save nominal disparity image
	  filePath.str(std::string());
	  filePath << outputDirectoryPath_ << "nominal_disparity/" << disparityErrorImage_.header.stamp << "_nom.csv";

	  fileStream.close();
	  fileStream.open(filePath.str().c_str());
	  if(!fileStream.is_open())
	  {
	    ROS_ERROR_STREAM("Could not open output file.");
	    return false;
	  }

	  data_ptr = (float*)nominalDisparityImage_.image.data.data();
	  for(unsigned int v = 0; v < disparityImage_.image.height; v++)
	  {
	    for(unsigned int u = 0; u < disparityImage_.image.width; u++)
	    {
	      fileStream << std::scientific << std::setprecision(6) << data_ptr[disparityImage_.image.width*v + u] << "\t";
	    }
	    fileStream << std::endl;
	  }

	  //Save plane parameters
	  filePath.str(std::string());
	  filePath << outputDirectoryPath_ << "planeParams.csv";

	  fileStream.close();
	  fileStream.open(filePath.str().c_str(), std::ios_base::app);
	  if(!fileStream.is_open())
	  {
	    ROS_ERROR_STREAM("Could not open output file.");
	    return false;
	  }

	  fileStream << disparityErrorImage_.header.stamp;
	  for(int i = 0; i < 4; i++)
	    fileStream << "\t" << std::setprecision(6) << planeParameters_[i];
	  fileStream << std::endl;

	  //Save disparity error image
	  filePath.str(std::string());
	  filePath << outputDirectoryPath_ << "disparity_error/" << disparityErrorImage_.header.stamp << ".csv";

	  fileStream.open(filePath.str().c_str());
	  if(!fileStream.is_open())
	  {
	    ROS_INFO("Could not open output file.");
	    return false;
	  }

	  data_ptr = (float*)disparityErrorImage_.image.data.data();
	  for(unsigned int v = 0; v < disparityErrorImage_.image.height; v++)
	  {
	    for(unsigned int u = 0; u < disparityErrorImage_.image.width; u++)
	    {
	      fileStream << std::scientific << std::setprecision(6) << data_ptr[disparityErrorImage_.image.width*v + u] << "\t";
	    }
	    fileStream << std::endl;
	  }
	}
	return true;
}

bool DisparityErrorProcessor::cropPointCloud(const sensor_msgs::PointCloud2ConstPtr& pointCloud2,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloudOutput) const
{
	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(*pointCloud2, pcl_pc);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromPCLPointCloud2(pcl_pc, *cloud);

	int width = xCropMax_ - xCropMin_;
	int height = yCropMax_ - yCropMin_;

	//Make sure the new cloud is smaller than the old cloud
	if(width < 1 || height < 1 || xCropMax_ > cloud->width || yCropMax_ > cloud->height)
	{
		ROS_ERROR_STREAM("ERROR: Invalid cropping parameters.");
		return false;
	}

	//If output cloud and input cloud are equally large, the input cloud is just copied.
	if(width == cloud->width && height == cloud->height)
	{
		pointCloudOutput.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		return true;
	}

	//Create the new cloud
	pointCloudOutput.reset(new pcl::PointCloud<pcl::PointXYZ>(width, height));

	//Copy data from the center rectangle of the old cloud to the new cloud
	int uIn = xCropMin_;
	int vIn0 = yCropMin_;
	int vIn;

	for(int u = 0; u < width; u++)
	{
		vIn = vIn0;
		for(int v = 0; v < height; v++)
		{
			pointCloudOutput->at(u, v) = cloud->at(uIn, vIn);
			vIn++;
		}
		uIn++;
	}

	pointCloudOutput->header = cloud->header;
	pointCloudOutput->is_dense = false;

	return true;
}

