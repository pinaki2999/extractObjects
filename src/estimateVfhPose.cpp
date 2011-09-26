/*
 * extractObject.cpp
 *
 *  Created on: Sep 25, 2011
 *      Author: reon
 */

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <ros/publisher.h>
#include <time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
#include <pcl/features/vfh.h>




class vfhPoseEstimator{

private:

    int noOfNeighbors;
    float threshold;

public:

/** estimates poses of received cube-data*/
void objectClusterCallback(const sensor_msgs::PointCloud2 &cloud){

}


/** Initializes the pose estimator.
* Reads the saved kd-tree of the training data and loads it
* \param noOfneighbors No of nearest poses to be estimated
* \param threshold Distance Threshold for the NN search
*/
void initialize(int noOfNeighbors, float threshold){

    this->noOfNeighbors = noOfNeighbors;
    this->threshold = threshold;




}

};


int main(int argc, char* argv[]) {


	ros::init(argc, argv, "estimateVfhPose");
	ros::NodeHandle nh;

    int noOfObjects = 1;
    int noOfNeighbors= 2;
    float distanceThreshold = 10;

    if(argc>1) noOfObjects = atoi(argv[1]);
    if(argc>2) noOfNeighbors= atoi(argv[2]);
    if(argc==3) distanceThreshold= atof(argv[3]);

    vfhPoseEstimator poseEstimator;
    poseEstimator.initialize( noOfNeighbors, distanceThreshold );

    ros::Subscriber  objectClusterSubscriber[noOfObjects];
    for (int i =0; i<noOfObjects; i++ ) {
      std::stringstream ss;
      ss << "object_cluster_" << i+1;
      objectClusterSubscriber[i]  = nh.subscribe(ss.str(), 1, &vfhPoseEstimator::objectClusterCallback, &poseEstimator);
    }


    ROS_INFO("Now estimating Object Poses ;)");

	ros::spin();
	return 0;

}
