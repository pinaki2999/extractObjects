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
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>

#include <vector>

#include <extractObjects/extractObjectsConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace std;
class icpPoseEstimator{

private:

pcl::PointCloud<pcl::PointXYZ>  cubeModel3D;
pcl::PointCloud<pcl::PointXYZ>  cubeModel2D;


float cubeSideLength;
float lastCentroid[3];
float last_min_depth;


ros::Publisher *publisher;

int *neighbors;
double *distance;
double best_score;

void generateCube3D(){

	int numOfFaces=3;
	int cloudWidthEachFace = 5;

    if((cloudWidthEachFace) % 2 != 0) cloudWidthEachFace+=1;

	// Fill in the cloud data
	cubeModel3D.width    = (cloudWidthEachFace*cloudWidthEachFace)*numOfFaces;
	cubeModel3D.height   = 1;
	cubeModel3D.is_dense = false;
	cubeModel3D.points.resize (cubeModel3D.width * cubeModel3D.height);

	float incr = cubeSideLength/cloudWidthEachFace;
	float xincr, yincr, zincr;
    float origin[3];
    origin[0]=-cubeSideLength/2.0;
    origin[1]=-cubeSideLength/2.0;
    origin[2]=-cubeSideLength/2.0;
	int pointIndex=0;

	for (int face=0; face < numOfFaces; face++) {
		switch(face){

		case 0:
			//xy-plane
			xincr = origin[0];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < cloudWidthEachFace; i++){
				yincr = origin[1];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < cloudWidthEachFace; j++) {
					cubeModel3D.points[pointIndex].x = xincr;
					cubeModel3D.points[pointIndex].y = yincr;
					cubeModel3D.points[pointIndex].z = origin[2]+cubeSideLength;
					pointIndex++;
					yincr += incr;

				}
				xincr += incr;
			}
			break;

		case 1:
			//yz-plane
			yincr = origin[1];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < cloudWidthEachFace; i++){
				zincr = origin[2];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < cloudWidthEachFace; j++) {
					cubeModel3D.points[pointIndex].x = origin[0];
					cubeModel3D.points[pointIndex].y = yincr;
					cubeModel3D.points[pointIndex].z = zincr;
					pointIndex++;
					zincr += incr;
				}
				yincr += incr;
			}
			break;

		case 2:
			//xz-plane
			xincr = origin[0];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < cloudWidthEachFace; i++){
				zincr = origin[2];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < cloudWidthEachFace; j++) {
					cubeModel3D.points[pointIndex].x = xincr;
					cubeModel3D.points[pointIndex].y = origin[1];
					cubeModel3D.points[pointIndex].z = zincr;
					pointIndex++;
					zincr += incr;
				}
				xincr += incr;
			}

			break;

		default:
			break;
		}
	}



	float tempHomogenousMatrix[16];
	calculateHomogeneousMatrix(90,0,0,0,0,0,tempHomogenousMatrix,true);
    applyHomogeneousTransformation(&cubeModel3D, &cubeModel3D, tempHomogenousMatrix);
    applyHomogeneousTransformation(&cubeModel3D, &cubeModel3D, tempHomogenousMatrix);

}



void generateCube2D(){

	int numOfFaces=2;
	int cloudWidthEachFace = 5;

    if((cloudWidthEachFace) % 2 != 0) cloudWidthEachFace+=1;

	// Fill in the cloud data
	cubeModel2D.width    = (cloudWidthEachFace*cloudWidthEachFace)*numOfFaces;
	cubeModel2D.height   = 1;
	cubeModel2D.is_dense = false;
	cubeModel2D.points.resize (cubeModel2D.width * cubeModel2D.height);

	float incr = cubeSideLength/cloudWidthEachFace;
	float xincr, yincr, zincr;
    float origin[3];
    origin[0]=-cubeSideLength/2.0;
    origin[1]=-cubeSideLength/2.0;
    origin[2]=-cubeSideLength/2.0;
	int pointIndex=0;

	for (int face=0; face < numOfFaces; face++) {
		switch(face){

		case 0:
			//xy-plane
			xincr = origin[0];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < cloudWidthEachFace; i++){
				yincr = origin[1];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < cloudWidthEachFace; j++) {
					cubeModel2D.points[pointIndex].x = xincr;
					cubeModel2D.points[pointIndex].y = yincr;
					cubeModel2D.points[pointIndex].z = origin[2]+cubeSideLength;
					pointIndex++;
					yincr += incr;

				}
				xincr += incr;
			}
			break;

		case 1:
			//yz-plane
			yincr = origin[1];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < cloudWidthEachFace; i++){
				zincr = origin[2];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < cloudWidthEachFace; j++) {
					cubeModel2D.points[pointIndex].x = origin[0];
					cubeModel2D.points[pointIndex].y = yincr;
					cubeModel2D.points[pointIndex].z = zincr;
					pointIndex++;
					zincr += incr;
				}
				yincr += incr;
			}
			break;

		case 2:
			//xz-plane
			xincr = origin[0];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < cloudWidthEachFace; i++){
				zincr = origin[2];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < cloudWidthEachFace; j++) {
					cubeModel2D.points[pointIndex].x = xincr;
					cubeModel2D.points[pointIndex].y = origin[1];
					cubeModel2D.points[pointIndex].z = zincr;
					pointIndex++;
					zincr += incr;
				}
				xincr += incr;
			}

			break;

		default:
			break;
		}
	}



	float tempHomogenousMatrix[16];
	calculateHomogeneousMatrix(90,0,0,0,0,0,tempHomogenousMatrix,true);
    applyHomogeneousTransformation(&cubeModel2D, &cubeModel2D, tempHomogenousMatrix);
    applyHomogeneousTransformation(&cubeModel2D, &cubeModel2D, tempHomogenousMatrix);

}


void applyHomogeneousTransformation(pcl::PointCloud<pcl::PointXYZ> * cloud, pcl::PointCloud<pcl::PointXYZ>*
transformedCloudCube, float *homogenousMatrix){
	transformedCloudCube->width = cloud->width;
	transformedCloudCube->height = cloud->height;
	transformedCloudCube->is_dense = false;
	transformedCloudCube->points.resize (transformedCloudCube->width * transformedCloudCube->height);
	/*
	 * layout:
	 * 0 4 8  12
	 * 1 5 9  13
	 * 2 6 10 14
	 * 3 7 11 15
	 */
	/* rotate */

	for (unsigned int i = 0; i < cloud->size(); i++ ) {
		float xTemp,yTemp,zTemp;
		xTemp = (cloud->points[i].x * homogenousMatrix[0] + cloud->points[i].y * homogenousMatrix[4] +
				cloud->points[i].z * homogenousMatrix[8]);
		yTemp = cloud->points[i].x * homogenousMatrix[1] + cloud->points[i].y * homogenousMatrix[5] +
				cloud->points[i].z * homogenousMatrix[9];
		zTemp = cloud->points[i].x * homogenousMatrix[2] + cloud->points[i].y * homogenousMatrix[6] +
				cloud->points[i].z * homogenousMatrix[10];

		/* translate */
		transformedCloudCube->points[i].x = (xTemp + homogenousMatrix[12]);
		transformedCloudCube->points[i].y = (yTemp + homogenousMatrix[13]);
		transformedCloudCube->points[i].z = (zTemp + homogenousMatrix[14]);
	}



}



void calculateHomogeneousMatrix(float xRot,float yRot,
		float zRot, float xtrans, float ytrans, float ztrans, float* homogeneousMatrix, bool inDegrees){
	if(inDegrees){
		float PI = 3.14159265;
		xRot = xRot*(PI/180);
		yRot *= yRot*(PI/180);
		zRot *= zRot*(PI/180);
	}
	/*
	 * layout:
	 * 0 4 8  12
	 * 1 5 9  13
	 * 2 6 10 14
	 * 3 7 11 15
	 */


	homogeneousMatrix[3]=0;
	homogeneousMatrix[7]=0;
	homogeneousMatrix[11]=0;
	homogeneousMatrix[15]=1;

    //translation
    homogeneousMatrix[12]=xtrans;
	homogeneousMatrix[13]=ytrans;
	homogeneousMatrix[14]=ztrans;

	//rotation
	homogeneousMatrix[0] = cos(yRot)*cos(zRot);
	homogeneousMatrix[1] = cos(yRot)*sin(zRot);
	homogeneousMatrix[2] = -sin(yRot);

	homogeneousMatrix[4] = -cos(xRot)*sin(zRot) + sin(xRot)*sin(yRot)*cos(zRot);
	homogeneousMatrix[5] = cos(xRot)*cos(zRot) + sin(xRot)*sin(yRot)*sin(zRot);
	homogeneousMatrix[6] = sin(xRot)*cos(yRot);

	homogeneousMatrix[8] = sin(xRot)*sin(zRot) + cos(xRot)*sin(yRot)*cos(zRot);
	homogeneousMatrix[9] = -sin(xRot)*cos(zRot) + cos(xRot)*sin(yRot)*sin(zRot);
	homogeneousMatrix[10]= cos(xRot)*cos(yRot);


}



public:




/** estimates poses of received cube-data*/
void objectClusterCallback(const sensor_msgs::PointCloud2 &cloud){

ROS_INFO("recieved an object cluster");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    float xTrans=0.0, yTrans=0.0, zTrans=0.0, xRot=0.0, yRot=0.0, zRot=0.0;
    pcl::fromROSMsg (cloud, *cloud_xyz_ptr);


    /**Get the orientation and translation of the objects*/

            Eigen::Vector4f centroid3d;
        	pcl::compute3DCentroid(*cloud_xyz_ptr,centroid3d);

            xTrans = centroid3d[0];
            yTrans = centroid3d[1];
            zTrans = centroid3d[2];
            ROS_INFO("Best estimate \n\tTranslation-[x,y,z,]=[%f,%f,%f]",xTrans, yTrans, zTrans);


     /**Check if the camera has moved or not. If true the centroid of the point clouds will be shifted*/

            float centroid_shift_distance = sqrt ((lastCentroid[0]-xTrans)*(lastCentroid[0]-xTrans) +
                                             (lastCentroid[1]-yTrans)*(lastCentroid[1]-yTrans) +
                                              (lastCentroid[2]-zTrans)*(lastCentroid[2]-zTrans) ) ;
            float epsilon = 0.001;
            ROS_INFO("Centroid Shift Distance: %f", centroid_shift_distance);


            if( centroid_shift_distance > epsilon ) {
                best_score = DBL_MAX;
                lastCentroid[0] = xTrans;
                lastCentroid[1] = yTrans;
                lastCentroid[2] = zTrans;
            }
       /**Translate the cube models which will be our initial estimate for ICP*/
                float homogeneousMatrix[16];
                pcl::PointCloud<pcl::PointXYZ> transformedCubeModel3D;
                pcl::PointCloud<pcl::PointXYZ> transformedCubeModel2D;
                calculateHomogeneousMatrix(0,0,0,xTrans,yTrans,zTrans,homogeneousMatrix,true);
                //calculateHomogeneousMatrix(0,0,0,0,0,0,homogeneousMatrix,true);
                applyHomogeneousTransformation(&cubeModel3D, &transformedCubeModel3D, homogeneousMatrix);
                applyHomogeneousTransformation(&cubeModel2D, &transformedCubeModel2D, homogeneousMatrix);



        /** Perform registration of the models using ICP for the 3D model*/

                pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp3D;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ> Final3D;

                cloud_in_ptr->width = transformedCubeModel3D.width;
                cloud_in_ptr->height = transformedCubeModel3D.height;
                cloud_in_ptr->points.resize(cloud_in_ptr->width * cloud_in_ptr->height);

                for(unsigned int i=0; i < cloud_in_ptr->points.size();i++){

                    cloud_in_ptr->points[i].x = transformedCubeModel3D.points[i].x;
                    cloud_in_ptr->points[i].y = transformedCubeModel3D.points[i].y;
                    cloud_in_ptr->points[i].z = transformedCubeModel3D.points[i].z;
                }

                icp3D.setInputCloud(cloud_in_ptr);
                icp3D.setInputTarget(cloud_xyz_ptr);
                icp3D.setMaxCorrespondenceDistance(*distance);

                  icp3D.setTransformationEpsilon (1e-6);
                icp3D.setMaximumIterations (50);


                icp3D.align(Final3D);

                //std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
                ROS_INFO("Used Distance %f",*distance);

               // icp.estimateRigidTransformationLM(cloud_in_ptr,cloud_xyz_ptr, transformation_matrix);

                //std::cout << icp.getFinalTransformation() << std::endl;



        /** Perform registration of the models using ICP for the 3D model*/

                pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp2D;
                pcl::PointCloud<pcl::PointXYZ> Final2D;

                cloud_in_ptr->width = transformedCubeModel2D.width;
                cloud_in_ptr->height = transformedCubeModel2D.height;
                cloud_in_ptr->points.resize(cloud_in_ptr->width * cloud_in_ptr->height);

                for(unsigned int i=0; i < cloud_in_ptr->points.size();i++){

                    cloud_in_ptr->points[i].x = transformedCubeModel2D.points[i].x;
                    cloud_in_ptr->points[i].y = transformedCubeModel2D.points[i].y;
                    cloud_in_ptr->points[i].z = transformedCubeModel2D.points[i].z;
                }

                icp2D.setInputCloud(cloud_in_ptr);
                icp2D.setInputTarget(cloud_xyz_ptr);
                icp2D.setMaxCorrespondenceDistance(*distance);

                  icp2D.setTransformationEpsilon (1e-6);
                icp2D.setMaximumIterations (1000);


                icp2D.align(Final2D);

                //std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
                ROS_INFO("Used Distance %f",*distance);

               // icp.estimateRigidTransformationLM(cloud_in_ptr,cloud_xyz_ptr, transformation_matrix);

                //std::cout << icp.getFinalTransformation() << std::endl;


                if( icp2D.getFitnessScore() < icp3D.getFitnessScore()) {

                    if (icp2D.getFitnessScore() < best_score ){
                        Final2D.header.frame_id = "openni_rgb_optical_frame";
                        publisher->publish(Final2D);
                        best_score = icp2D.getFitnessScore();
                    } else {
                        /** Publish previous best transform*/
                    }

                } else {

                    if (icp3D.getFitnessScore() < best_score ) {
                        Final3D.header.frame_id = "openni_rgb_optical_frame";
                        publisher->publish(Final3D);
                        best_score = icp3D.getFitnessScore();
                    }else {
                        /** Publish previous best transform*/
                    }

                }



}



/** Initializes the pose estimator.
* Reads the saved kd-tree of the training data and loads it
*/
int initialize(float cubeSideLength, ros::Publisher *publisher, int *neighbors, double *distance){

    best_score = DBL_MAX;

    lastCentroid[0] = 0.0;
    lastCentroid[1] = 0.0;
    lastCentroid[2] = 0.0;

    last_min_depth = 0.0;

    this->cubeSideLength = cubeSideLength;
    this->publisher = publisher;

    this->distance = distance;
    this->neighbors = neighbors;

    generateCube3D();
    generateCube2D();
    return 1;

}



};




double distances;
int neighbors;



/**
*Callback for dynamic reconfigure
*/
void callback(extractObjects::extractObjectsConfig &config, uint32_t level)
{
	distances = config.distance;
	neighbors = config.neighbors;
}

int main(int argc, char* argv[]) {


	ros::init(argc, argv, "icpPoseEstimator");
	ros::NodeHandle nh;
    distances = 4;
    int noOfObjects = 1;                //by default it will estimate poses for only one object
    float cubeSideLength = 0.08;

    if(argc>1) noOfObjects = atoi(argv[1]);
    if(argc>2) cubeSideLength = atof(argv[2]);

    icpPoseEstimator poseEstimator[noOfObjects];
    ros::Subscriber  objectClusterSubscriber[noOfObjects];
    ros::Publisher cubeModelPublishers[noOfObjects];


    for (int i =0; i<noOfObjects; i++ ) {
      std::stringstream ss, ss_1;
      ss << "object_cluster_" << i+1;
      ss_1 << "cube_model_" << i+1;
      cubeModelPublishers[i] = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >  (ss_1.str(), 1);
      poseEstimator[i].initialize( cubeSideLength , &cubeModelPublishers[i], &neighbors, &distances);
      objectClusterSubscriber[i]  = nh.subscribe(ss.str(), 1, &icpPoseEstimator::objectClusterCallback, &poseEstimator[i]);
      //objectClusterSubscriber[i]  = nh.subscribe("/camera/rgb/points", 1, &icpPoseEstimator::objectClusterCallback, &poseEstimator[i]);

    }


	dynamic_reconfigure::Server<extractObjects::extractObjectsConfig> srv;
	dynamic_reconfigure::Server<extractObjects::extractObjectsConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	srv.setCallback(f);


    ROS_INFO("Now estimating Object Poses using ICP;)");





/*    string ch = "y";
    while( !ch.compare("y")) {
-*/
	ros::spin();
/*
    cout << "Do you want to continue ? "<< endl;
    cin >> ch;
    }
*/	return 0;

}


//------------------------------------------------------------------------------------------------------------



