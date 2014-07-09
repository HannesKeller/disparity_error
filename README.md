Disparity Error Calculation
======================

Overview
---------------

This [ROS] package was used to develop a model of the disparity error of a stereo camera. It provides functionalities for fitting planes to point clouds and for storing
disparity data in tab-separated .csv files for further processing.

**Author: Hannes Keller, kellerh@student.ethz.ch<br />
Affiliation: Autonomous Systems Lab, ETH Zurich**

Installation
------------

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionaly, the disparity error calculator depends on following software:

- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing).


### Building

In order to install the Robot-Centric Elevation Mapper, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd catkin_workspace/src
    git clone https://github.com/ethz-asl/elevation_mapping.git
    cd ../
    catkin_make


Basic Usage
------------
The disparity error calculator can be configured using the launch file `disparity_error/launch/disparityError.launch`. See the **Parameters** section for more information.


Nodes
------------

### Node: disparityErrorNode

This is the only node present in the package. It has two main callbacks - one for point clouds and one for disparity images. </br>
<p>
**Point cloud callback** <br/>
This callback is only active if plane fitting is enabled. It crops the point cloud to a rectangular section according to the specified parameters and then fits a plane
to the cropped point cloud using RANSAC. The resulting plane parameters are published as a [shape_msgs/Plane].
</p>
<p>
**Disparity callback** <br />
This callback provides mainly file exporting functionalities. If file output is enabled (see **Services**), it stores the received disparity image to a tab-separated .csv file.
If plane fitting is enabled, it also computes a nominal disparity for the fitted plane and the absolute difference between the received disparity and the computed nominal disparity. Both of these images get saved as well. In addition, the plane parameters are also stored in a file.
</p>

#### Subscribed Topics

* **`/points2`** ([sensor_msgs/PointCloud2])

    The point cloud.
    
* **`/disparity`** ([stereo_msgs/DisparityImage])

    The disparity image.
    
* **`/cam0/camera_info`** ([sensor_msgs/CameraInfo])

    Camera info of the left camera.
    
* **`/cam1/camera_info`** ([sensor_msgs/CameraInfo])

    Camera info of the right camera.



#### Published Topics

* **`/fitted_plane`** ([shape_msgs/Plane])

    The plane fitted to the point cloud.
    
* **`/disparity_error`** ([stereo_msgs/DisparityImage])

    The difference between nominal and measured disparity.


#### Services

* **`toggleFileOutput`** ([std_srvs/Empty])

    Toggle file output for disparity images.

        rosservice call /disparityErrorNode/toggleFileOutput


#### Parameters

        <param name="outputDirectoryPath" value="/home/hannes/Documents/Semesterprojekt/Experiments/move_70_2/" />
        <param name="x_crop_min" value="135" />
        <param name="x_crop_max" value="744" />
        <param name="y_crop_min" value="7" />
        <param name="y_crop_max" value="472" />
        <param name="plane_fitting_threshold" value="0.05" />
        <param name="enable_plane_fitting" value="false" />

* **`outputDirectoryPath`** (string, default: "/home/")
 
    The path to the desired file output directory. This directory should contain folders called **disparity**, **disparity_error** and **nominal_disparity**.

* **`x_crop_min`** (int, default: 0)
 
    Left border for point cloud cropping.
    
* **`x_crop_max`** (int, default: 0)
 
    Right border for point cloud cropping.

* **`y_crop_min`** (int, default: 0)
 
    Top border for point cloud cropping.

* **`y_crop_max`** (int, default: 0)
 
    Bottom border for point cloud cropping.
    
* **`plane_fitting_threshold`** (int, default: 0.01)
 
    Threshold for the RANSAC algorithm.
    
* **`enable_plane_fitting`** (bool, default: false)
 
    Plane fitting toggle.


[ROS]: http://www.ros.org
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[sensor_msgs/CameraInfo]: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
[stereo_msgs/DisparityImage]: http://docs.ros.org/api/stereo_msgs/html/msg/DisparityImage.html
[geometry_msgs/PoseWithCovarianceStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[std_srvs/Empty]: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
[shape_msgs/Plane]: http://docs.ros.org/api/shape_msgs/html/msg/Plane.html

