/*
 * extractObject.cpp
 *
 *  Created on: Sep 19, 2011
 *      Author: reon
 */

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <ros/publisher.h>

#include <pcl/filters/passthrough.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

#include <time.h>
#include <sstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include "fstream"


#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


using namespace std;
/**
 * Global variables
 */

class objectClusterExtractor{

private:
ros::Publisher *pubClusters;
int noOfObjects;

public:
	/**
	 * gets the raw depth-rgb data of the extracted ROI and finds the object clusters in the data
	 */
	void hsvBasedROICallback(const sensor_msgs::PointCloud2 &cloud){

        ROS_INFO("Extracting clusters");

		pcl::PointCloud<pcl::PointXYZ>::Ptr hsv_extracted_roi(new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::fromROSMsg (cloud, *hsv_extracted_roi);

        // Creating the KdTree object for the search method of the extraction
        pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
        tree->setInputCloud (hsv_extracted_roi);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClusterExtractor;
        euclideanClusterExtractor.setClusterTolerance (0.01); // 2cm
        euclideanClusterExtractor.setMinClusterSize (500);
        euclideanClusterExtractor.setMaxClusterSize (25000);
        euclideanClusterExtractor.setSearchMethod (tree);
        euclideanClusterExtractor.setInputCloud(hsv_extracted_roi);
        euclideanClusterExtractor.extract (cluster_indices);

        ROS_INFO("Number of objects found: %d", cluster_indices.size());
        int i=0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          if(i<noOfObjects){
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster->points.push_back (hsv_extracted_roi->points[*pit]); //*

                //          std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

                //Publish the extracted object

                cloud_cluster->header.frame_id = "openni_rgb_optical_frame";
                pubClusters[i].publish(*cloud_cluster);
            i++;
          }
        }


	}

	void setPublishers(ros::Publisher *publishers){
        pubClusters = publishers;
	}

    void setNumberOfObjects(int noOfObjects){
        this->noOfObjects = noOfObjects;
	}


};

int main(int argc, char* argv[]){

    int noOfObjects = 1;

    if(argc>1) noOfObjects = atoi(argv[1]);

	ros::init(argc, argv, "extractObjectClusters");
	ros::NodeHandle nh;
    ros::Publisher clusterPublishers[noOfObjects];

    objectClusterExtractor clusterExtractor;

    for (int i =0; i<noOfObjects; i++ ) {
      std::stringstream ss;
      ss << "object_cluster_" << i+1;
      clusterPublishers[i] = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >  (ss.str(), 1);
    }

    clusterExtractor.setPublishers(clusterPublishers);
    clusterExtractor.setNumberOfObjects(noOfObjects);
    ros::Subscriber  kinectCloudRaw = nh.subscribe("extractedROI", 1, &objectClusterExtractor::hsvBasedROICallback, &clusterExtractor);
    ROS_INFO("Now extracting object clusters ;)");

	ros::spin();
	return 0;
};
