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
	void rgbToHsv(int *rgb, float *hsv){
		float r = rgb[0];
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

     //   ROS_INFO("rgbToHsv: %f,%f,%f\n", hsv[0]  , hsv[1], hsv[2]);

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



		int cloudSize=0;;
		cloudSize= cloud_xyz_rgb_ptr->size();
		int rgb[cloudSize][3];
		float hsv[cloudSize][3];
        //ROS_INFO("size of input cloud %d", cloud_xyz_rgb->size());
		//-------------------------------extracting the r,g,b values
/**Original**/
/**---------------------------------------------------------------------------------------*/
		for(int i=0; i<cloudSize; i++){//i<10; i++){//
			int rgb_val = *reinterpret_cast<int*>(&cloud_xyz_rgb_ptr->points[i].rgb);
			rgb[i][0] = ((rgb_val >> 16) & 0xff);
			rgb[i][1] = ((rgb_val >> 8) & 0xff);
			rgb[i][2] = (rgb_val & 0xff);
            //converting to hsv spectrum
			rgbToHsv(rgb[i],hsv[i]);
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

    }

    void setHsvLimits(float minH, float maxH,float minS, float maxS){
        this->limitH[0] = minH;
        this->limitH[1] = maxH;
        this->limitS[0] = minS;
        this->limitS[1] = maxS;
    }
};

int main(int argc, char* argv[]){
	ros::init(argc, argv, "extractObject");
	ros::NodeHandle nh;
    ros::Publisher pubROI = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >  ("extractedROI", 1);




    cubeGrabber greenCubeGrabber;
	greenCubeGrabber.setPublisher(&pubROI);

	float minH=0, maxH=255, minS=0, maxS=255;
    if (argc == 2) {
        ifstream configFileStream;
        configFileStream.open(argv[1]);
        if ( configFileStream.is_open() ) {     //if file exists
            string s;
            while(getline(configFileStream, s)){    //extract the values of the parameters
                	std::vector< std::string > tempVec;
                    boost::split(tempVec, s, boost::is_any_of("="));
                    if(!tempVec[0].compare("minH")) {
                         minH = atof(tempVec[1].c_str());
                    } else if(!tempVec[0].compare("maxH")) {
                         maxH = atof(tempVec[1].c_str());
                    } else if(!tempVec[0].compare("minS")) {
                         minS = atof(tempVec[1].c_str());
                    } else if(!tempVec[0].compare("maxS")) {
                         maxS = atof(tempVec[1].c_str());
                    }
            }

        }
    }

    greenCubeGrabber.setHsvLimits(minH,maxH,minS,maxS);
    ROS_INFO ("ROI extraction using minH=%f, maxH=%f, minS=%f, maxH=%f",minH,maxH,minS,maxS);

    ros::Subscriber  kinectCloudRaw = nh.subscribe("/camera/rgb/points", 1,&cubeGrabber::kinectCloudRawCallback, &greenCubeGrabber);

    ROS_INFO("Now extracting ROIs ;)");

	ros::spin();
	return 0;
};
