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


#include <time.h>
#include <sstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include "fstream"

using namespace std;
/**
 * Global variables
 */

class cubeGrabber{

public:

	char timestamp[20];
	ros::Publisher pubROI;
    float limitH[2], limitS[2];
    int rgb[640*480][3];
	float hsv[640*480][3];
    int rgb_val;
    int cloudSize;
    float max,min,r,g,b;
    float epsilon;

    /**
	 * finds min of three float values
	 **/
	float minVal(float a, float b, float c){

		if(a < b){
			if(a < c){
				return a;
			} else {
				return c;
			}
		} else {
			if(b < c){
				return b;
			} else {
				return c;
			}
		}

	}



	/**
	 * finds max of three float values
	 **/
	float maxVal(float a, float b, float c){

		if(a > b){
			if(a > c){
				return a;
			} else {
				return c;
			}
		} else {
			if(b > c){
				return b;
			} else {
				return c;
			}
		}

	}

	/** In case of 8-bit and 16-bit images
        R, G and B are converted to floating-point format and scaled to fit 0..1 range

        V <- max(R,G,B)
        S <- (V-min(R,G,B))/V   if V≠0, 0 otherwise

                (G - B)*60/S,  if V=R
        H <-    180+(B - R)*60/S,  if V=G
                240+(R - G)*60/S,  if V=B

        if H<0 then H<-H+360

        On output 0≤V≤1, 0≤S≤1, 0≤H≤360.
        The values are then converted to the destination data type:
        8-bit images:
            V <- V*255, S <- S*255, H <- H/2 (to fit to 0..255)
        16-bit images (currently not supported):
            V <- V*65535, S <- S*65535, H <- H
        32-bit images:
            H, S, V are left as is
	 */
	void rgbToHsv(int i){
		/*float r = rgb[0];
		r = r / 255.0;
		float g = rgb[1];
		g = g / 255.0;
		float b = rgb[2];
		b = b / 255.0;

		float max = maxVal(r, g, b);
		float min = minVal(r, g, b);

		hsv[2] = max;

		if(hsv[2]!=0){
			hsv[1]= (hsv[2]-min)/hsv[2];
		} else {
			hsv[1]=0;
		}
		float epsilon = 0.00001;
		if(fabs(hsv[2]-r) <= epsilon){
			hsv[0]= (g-b)*60.0/hsv[1];
		} else if(fabs(hsv[2]-g) <= epsilon) {
			hsv[0]= 180.0+(b-r)*60.0/hsv[1];

		} else if(fabs(hsv[2]-b) <= epsilon) {
			hsv[0]=240+(r - g)*60.0/hsv[1];
		}

		if(hsv[0]<0.0)hsv[0]+=360.0;

		hsv[0] /= 2.0;
		hsv[1] *= 255.0;
		hsv[2] *= 255.0;
*/
     //   ROS_INFO("rgbToHsv: %f,%f,%f\n", hsv[0]  , hsv[1], hsv[2]);
		r = rgb[i][0]/255.0;
		g = rgb[i][1]/255.0;
		b = rgb[i][2]/255.0;


		max = maxVal(r, g, b);
		min = minVal(r, g, b);

		hsv[i][2] = max;

		if(hsv[i][2]!=0){
			hsv[i][1]= (hsv[i][2]-min)/hsv[i][2];
		} else {
			hsv[i][1]=0;
		}
		if(fabs(hsv[i][2]-r) <= epsilon){
			hsv[i][0]= (g-b)*60.0/hsv[i][1];
		} else if(fabs(hsv[i][2]-g) <= epsilon) {
			hsv[i][0]= 180.0+(b-r)*60.0/hsv[i][1];

		} else if(fabs(hsv[i][2]-b) <= epsilon) {
			hsv[i][0]=240+(r - g)*60.0/hsv[i][1];
		}

		if(hsv[i][0]<0.0)hsv[i][0]+=360.0;

		hsv[i][0] /= 2.0;
		hsv[i][1] *= 255.0;
		hsv[i][2] *= 255.0;

	}


	/**
	 * Returns indices of points which are under permissible HSV limits
	 *
	 **/
	void extractHsvBasedROI(float hsv[][3], int cloudSize, pcl::PointIndices::Ptr inliers) {

		for (int i = 0; i < cloudSize; i++){

			//ROS_INFO("Comparing hsv[%d][0] = %f\n", i, hsv[i][0]);
			//ROS_INFO("Comparing hsv[%d][1] = %f\n", i, hsv[i][1]);

			if(	hsv[i][0] >=limitH[0] && hsv[i][0] <=limitH[1] &&
					hsv[i][1] >=limitS[0] && hsv[i][1] <=limitS[1] )
				inliers->indices.push_back(i);

		}
		//ROS_INFO("hsvBasedROI Inlier size: %d",inliers->indices.size());

	}


	/**
	 * gets the raw depth-rgb data
	 */
	void kinectCloudRawCallback(const sensor_msgs::PointCloud2 &cloud){



        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr hsv_extracted_roi_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg (cloud, *cloud_xyz_rgb_ptr);
        //ROS_INFO("\nrecieved a kinect_raw message...... :)");




		cloudSize= cloud_xyz_rgb_ptr->size();
        //ROS_INFO("size of input cloud %d", cloud_xyz_rgb->size());
		//-------------------------------extracting the r,g,b values
/**Original**/
/**---------------------------------------------------------------------------------------*/
		for(int i=0; i<cloudSize; i++){//i<10; i++){//
			rgb_val = *reinterpret_cast<int*>(&cloud_xyz_rgb_ptr->points[i].rgb);
			rgb[i][0] = ((rgb_val >> 16) & 0xff);
			rgb[i][1] = ((rgb_val >> 8) & 0xff);
			rgb[i][2] = (rgb_val & 0xff);
            //converting to hsv spectrum
			rgbToHsv(i);
		}

/**---------------------------------------------------------------------------------------*/

/**Experimental**/
/**---------------------------------------------------------------------------------------*/
/*		for(int i=0; i<cloudSize; i++){//i<10; i++){//
			int rgb_val = *reinterpret_cast<int*>(&cloud_xyz_rgb_ptr->points[i].rgb);
			hsv[i][0] = ((rgb_val >> 16) & 0xff);
			hsv[i][1] = ((rgb_val >> 8) & 0xff);
			hsv[i][2] = (rgb_val & 0xff);
            //converting to hsv spectrum
			//rgbToHsv(rgb[i],hsv[i]);
		}
*/
/**---------------------------------------------------------------------------------------*/


/**Original**/
/**---------------------------------------------------------------------------------------*/
        //extracting region of interests based on HSV values
        //ToDo optimize by using a single pointer only
        pcl::PointIndices::Ptr hsvBasedROI(new pcl::PointIndices());
        extractHsvBasedROI(hsv,cloudSize,hsvBasedROI);

        pcl::ExtractIndices<pcl::PointXYZRGB> extractHsvBasedROIcloud;
        // Extract the inliers
        extractHsvBasedROIcloud.setInputCloud (cloud_xyz_rgb_ptr);
        extractHsvBasedROIcloud.setIndices (hsvBasedROI);
        extractHsvBasedROIcloud.setNegative (false);
        extractHsvBasedROIcloud.filter (*hsv_extracted_roi_ptr);

/**---------------------------------------------------------------------------------------*/


/**Experimental**/
/**---------------------------------------------------------------------------------------*/
/*        hsv_extracted_roi_ptr->width=cloud_xyz_rgb_ptr->width;
        hsv_extracted_roi_ptr->height=cloud_xyz_rgb_ptr->height;
        hsv_extracted_roi_ptr->points.resize(hsv_extracted_roi_ptr->height * hsv_extracted_roi_ptr->width);

        for (int i = 0; i < cloudSize; i++){

			if(	hsv[i][0] >=limitH[0] && hsv[i][0] <=limitH[1] &&
					hsv[i][1] >=limitS[0] && hsv[i][1] <=limitS[1] ) {
                        hsv_extracted_roi_ptr->points[i].x = cloud_xyz_rgb_ptr->points[i].x ;
                        hsv_extracted_roi_ptr->points[i].y = cloud_xyz_rgb_ptr->points[i].y ;
                        hsv_extracted_roi_ptr->points[i].z = cloud_xyz_rgb_ptr->points[i].z ;
                        hsv_extracted_roi_ptr->points[i].rgb = cloud_xyz_rgb_ptr->points[i].rgb ;
					}
        }
*/
/**---------------------------------------------------------------------------------------*/


        //ROS_INFO("size of published cloud %d", hsv_extracted_roi->size());
        //Publish the extracted ROI

        hsv_extracted_roi_ptr->header.frame_id = "openni_rgb_optical_frame";
        pubROI.publish (hsv_extracted_roi_ptr);

	}



    /**
    * Set the publisher to output extracted ROIs
    */

    void setPublisher (ros::Publisher *pub){
        this->pubROI = *pub;
        epsilon = 0.00001;
    }

    void setHsvLimits(float minH, float maxH,float minS, float maxS){
        this->limitH[0] = minH;
        this->limitH[1] = maxH;
        this->limitS[0] = minS;
        this->limitS[1] = maxS;
    }
};

ros::Publisher pubGreenROI, pubRedROI, pubYellowROI;
cubeGrabber greenCubeGrabber, redCubeGrabber, yellowCubeGrabber;

	void kinectCloudRawCallback(const sensor_msgs::PointCloud2 &cloud){
        greenCubeGrabber.kinectCloudRawCallback(cloud);
        redCubeGrabber.kinectCloudRawCallback(cloud);
        yellowCubeGrabber.kinectCloudRawCallback(cloud);
	}


int main(int argc, char* argv[]){
	ros::init(argc, argv, "extractObject");
	ros::NodeHandle nh;

    pubGreenROI = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >  ("extractedROI_Green", 1);
    pubRedROI = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >  ("extractedROI_Red", 1);
    pubYellowROI = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >  ("extractedROI_Yellow", 1);

	greenCubeGrabber.setPublisher(&pubGreenROI);
	redCubeGrabber.setPublisher(&pubRedROI);
	yellowCubeGrabber.setPublisher(&pubYellowROI);

    ifstream configFileStream;

	float minH_Green=0, maxH_Green=255, minS_Green=0, maxS_Green=255;
    float minH_Red=0, maxH_Red=255, minS_Red=0, maxS_Red=255;
    float minH_Yellow=0, maxH_Yellow=255, minS_Yellow=0, maxS_Yellow=255;

        /**Reading Green ROI configuration*/
        configFileStream.open("./greenRoiConfig");
        if ( configFileStream.is_open() ) {     //if file exists
            string s;
            while(getline(configFileStream, s)){    //extract the values of the parameters
                	std::vector< std::string > tempVec;
                    boost::split(tempVec, s, boost::is_any_of("="));
                    if(!tempVec[0].compare("minH")) {
                         minH_Green = atof(tempVec[1].c_str());
                    } else if(!tempVec[0].compare("maxH")) {
                         maxH_Green = atof(tempVec[1].c_str());
                    } else if(!tempVec[0].compare("minS")) {
                         minS_Green = atof(tempVec[1].c_str());
                    } else if(!tempVec[0].compare("maxS")) {
                         maxS_Green = atof(tempVec[1].c_str());
                    }
            }

        }
        configFileStream.close();

        /**Reading Red ROI configuration*/
        configFileStream.open("./redRoiConfig");
        if ( configFileStream.is_open() ) {     //if file exists
            string s;
            while(getline(configFileStream, s)){    //extract the values of the parameters
                	std::vector< std::string > tempVec;
                    boost::split(tempVec, s, boost::is_any_of("="));
                    if(!tempVec[0].compare("minH")) {
                         minH_Red = atof(tempVec[1].c_str());
                    } else if(!tempVec[0].compare("maxH")) {
                         maxH_Red = atof(tempVec[1].c_str());
                    } else if(!tempVec[0].compare("minS")) {
                         minS_Red = atof(tempVec[1].c_str());
                    } else if(!tempVec[0].compare("maxS")) {
                         maxS_Red = atof(tempVec[1].c_str());
                    }
            }

        }
        configFileStream.close();



        /**Reading Yellow ROI configuration*/
        configFileStream.open("./yellowRoiConfig");
        if ( configFileStream.is_open() ) {     //if file exists
            string s;
            while(getline(configFileStream, s)){    //extract the values of the parameters
                	std::vector< std::string > tempVec;
                    boost::split(tempVec, s, boost::is_any_of("="));
                    if(!tempVec[0].compare("minH")) {
                         minH_Yellow = atof(tempVec[1].c_str());
                    } else if(!tempVec[0].compare("maxH")) {
                         maxH_Yellow = atof(tempVec[1].c_str());
                    } else if(!tempVec[0].compare("minS")) {
                         minS_Yellow = atof(tempVec[1].c_str());
                    } else if(!tempVec[0].compare("maxS")) {
                         maxS_Yellow = atof(tempVec[1].c_str());
                    }
            }

        }
        configFileStream.close();



    greenCubeGrabber.setHsvLimits(minH_Green,maxH_Green,minS_Green,maxS_Green);
    ROS_INFO ("Green ROI extraction using minH=%f, maxH=%f, minS=%f, maxH=%f",minH_Green,maxH_Green,minS_Green,maxS_Green);

    redCubeGrabber.setHsvLimits(minH_Red,maxH_Red,minS_Red,maxS_Red);
    ROS_INFO ("Red ROI extraction using minH=%f, maxH=%f, minS=%f, maxH=%f",minH_Red,maxH_Red,minS_Red,maxS_Red);

    yellowCubeGrabber.setHsvLimits(minH_Yellow,maxH_Yellow,minS_Yellow,maxS_Yellow);
    ROS_INFO ("Yellow ROI extraction using minH=%f, maxH=%f, minS=%f, maxH=%f",minH_Yellow,maxH_Yellow,minS_Yellow,maxS_Yellow);

    ros::Subscriber  kinectCloudRaw_green = nh.subscribe("/camera/rgb/points", 1,&kinectCloudRawCallback);

    ROS_INFO("Now extracting ROIs ;)");

	ros::spin();
	return 0;
};
