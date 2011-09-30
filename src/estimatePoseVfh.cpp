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
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>

#include <vector>

typedef std::pair<std::string, std::vector<float> > vfh_model;
using namespace std;
class vfhPoseEstimator{

private:

    int noOfNeighbors;
    float threshold;
    vfh_model histogram;    // The training model

    flann::Matrix<int> k_indices;
    flann::Matrix<float> k_distances;
    flann::Index<flann::ChiSquareDistance<float> > *index;
    std::vector<vfh_model> *models;
/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */

void
nearestKSearch (float vfhSignature[])
{

    std::vector<float> vec;
    vec.resize(308);
    for (int i =0; i < 308; i++){
    vec[i] = vfhSignature[i];
    }

 // Query point
    flann::Matrix<float> p = flann::Matrix<float>(new float[vec.size()], 1, vec.size());
    memcpy (&p.data[0], &vec[0], p.cols * p.rows * sizeof (float));


  k_indices = flann::Matrix<int>(new int[noOfNeighbors], 1, noOfNeighbors);
  k_distances = flann::Matrix<float>(new float[noOfNeighbors], 1, noOfNeighbors);
  ROS_INFO("%d",index->size());
  index->knnSearch (p, k_indices, k_distances, noOfNeighbors, flann::SearchParams (512));



 p.free();


}


void visualize(){
  // Load the results
  int k=noOfNeighbors;
  pcl::visualization::PCLVisualizer p ("VFH Cluster Classifier");
  int y_s = (int)floor (sqrt ((double)k));
  int x_s = y_s + (int)ceil ((k / (double)y_s) - y_s);
  double x_step = (double)(1 / (double)x_s);
  double y_step = (double)(1 / (double)y_s);
  pcl::console::print_highlight ("Preparing to load ");
  pcl::console::print_value ("%d", k);
  pcl::console::print_info (" files (");
  pcl::console::print_value ("%d", x_s);
  pcl::console::print_info ("x");
  pcl::console::print_value ("%d", y_s);
  pcl::console::print_info (" / ");
  pcl::console::print_value ("%f", x_step);
  pcl::console::print_info ("x");
  pcl::console::print_value ("%f", y_step);
  pcl::console::print_info (")\n");

  int viewport = 0, l = 0, m = 0;
  for (int i = 0; i < k; ++i)
  {
    std::string cloud_name = models->at (k_indices[0][i]).first;
    boost::replace_last (cloud_name, "_vfh", "");

    p.createViewPort (l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);
    l++;
    if (l >= x_s)
    {
      l = 0;
      m++;
    }

    sensor_msgs::PointCloud2 cloud;
    pcl::console::print_highlight (stderr, "Loading "); pcl::console::print_value (stderr, "%s ", cloud_name.c_str ());
    if (pcl::io::loadPCDFile (cloud_name, cloud) == -1)
      break;

    // Convert from blob to PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::fromROSMsg (cloud, cloud_xyz);

    if (cloud_xyz.points.size () == 0)
      break;

    pcl::console::print_info ("[done, ");
    pcl::console::print_value ("%d", (int)cloud_xyz.points.size ());
    pcl::console::print_info (" points]\n");
    pcl::console::print_info ("Available dimensions: ");
    pcl::console::print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

    // Demean the cloud
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (cloud_xyz, centroid);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_demean (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::demeanPointCloud<pcl::PointXYZ> (cloud_xyz, centroid, *cloud_xyz_demean);
    // Add to renderer*
    p.addPointCloud (cloud_xyz_demean, cloud_name, viewport);

    // Check if the model found is within our inlier tolerance
    std::stringstream ss;
    ss << k_distances[0][i];
    if (k_distances[0][i] > threshold)
    {
      p.addText (ss.str (), 20, 30, 1, 0, 0, ss.str (), viewport);  // display the text with red

      // Create a red line
      pcl::PointXYZ min_p, max_p;
      pcl::getMinMax3D (*cloud_xyz_demean, min_p, max_p);
      std::stringstream line_name;
      line_name << "line_" << i;
      p.addLine (min_p, max_p, 1, 0, 0, line_name.str (), viewport);
      p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, line_name.str (), viewport);
    }
    else
      p.addText (ss.str (), 20, 30, 0, 1, 0, ss.str (), viewport);

    // Increase the font size for the score*
    p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 18, ss.str (), viewport);

    // Add the cluster name
    p.addText (cloud_name, 20, 10, cloud_name, viewport);
  }
  // Add coordianate systems to all viewports
  p.addCoordinateSystem (0.1, 0);

    p.spin();
}


public:


/** estimates poses of received cube-data*/
void objectClusterCallback(const sensor_msgs::PointCloud2 &cloud){

    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    float xTrans=0.0, yTrans=0.0, zTrans=0.0, xRot=0.0, yRot=0.0, zRot=0.0;

    pcl::fromROSMsg (cloud, *cloud_xyz_ptr);

    /**Calculate Vfh signature of the point cloud*/
        // Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud ( cloud_xyz_ptr );
		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
		ne.setSearchMethod (tree);
		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch (0.03);
		// Compute the features
		ne.compute ( *cloud_normals );

		// Create the VFH estimation class, and pass the input dataset+normals to it
		pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
		vfh.setInputCloud ( cloud_xyz_ptr );
		vfh.setInputNormals ( cloud_normals );
		// alternatively, if cloud is of tpe PointNormal, do vfh.setInputNormals (cloud);

		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr treeVfh (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
		vfh.setSearchMethod (treeVfh);
        //Compute the features
		vfh.compute (*vfhs);

        ROS_INFO("Searching for neighbors now..");

    /**NN search*/
    nearestKSearch (vfhs->points[0].histogram);

    /**Output on screen*/
      // Output the results on screen
 /*   ROS_INFO ("The closest %d neighbors are:\n", noOfNeighbors);
      for (int i = 0; i < noOfNeighbors; ++i)
        ROS_INFO ("    %d - %s (%d) with a distance of: %f\n",
            i, models->at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
*/
    /**Visualize the corresponding training data*/
//  visualize();


    /**Get the orientation and translation of the objects*/

            Eigen::Vector4f centroid3d;
        	pcl::compute3DCentroid(*cloud_xyz_ptr,centroid3d);

            xTrans = centroid3d[0];
            yTrans = centroid3d[1];
            zTrans = centroid3d[2];

            std::vector< std::string > tempVec;
            ROS_INFO("|||||%s||||",models->at (k_indices[0][0]).first.c_str());
            boost::split(tempVec, models->at (k_indices[0][0]).first, boost::is_any_of("/"));
            ROS_INFO("%s",tempVec[tempVec.size()-1].c_str());
            boost::split(tempVec, tempVec[tempVec.size()-1], boost::is_any_of("_"));
            ROS_INFO("%s",tempVec[0].c_str());
            if(tempVec.size() >6) {
                    xRot = atof(tempVec[4].c_str());
                    yRot = atof(tempVec[5].c_str());
                    zRot = atof(tempVec[6].c_str());
            }

    ROS_INFO("Best estimate \n\tTranslation-[x,y,z,]=[%f,%f,%f] \n\tRotation-[x,y,z]=%f,%f,%f",xTrans, yTrans, zTrans, xRot, yRot, zRot);


}


/** Initializes the pose estimator.
* Reads the saved kd-tree of the training data and loads it
* \param noOfneighbors No of nearest poses to be estimated
* \param threshold Distance Threshold for the NN search
*/
int initialize(int noOfNeighbors, float threshold, flann::Index<flann::ChiSquareDistance<float> > *trainingIndex, std::vector<vfh_model> *trainingModels){

    this->noOfNeighbors = noOfNeighbors;
    this->threshold = threshold;
    this->index = trainingIndex;
    this->models = trainingModels;


return(1);

}

};

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
bool
loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
  ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    vfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}


int main(int argc, char* argv[]) {


	ros::init(argc, argv, "estimateVfhPose");
	ros::NodeHandle nh;

    int noOfObjects = 1;                //by default it will estimate poses for only one object
    int noOfNeighbors= 5;               //by default 5 nearest neighbours will be searched
    float distanceThreshold = DBL_MAX;  //no threshold

    if(argc>1) noOfObjects = atoi(argv[1]);
    if(argc>2) noOfNeighbors= atoi(argv[2]);
    if(argc==3) distanceThreshold= atof(argv[3]);




    // this won't be needed for flann > 1.6.10
    flann::ObjectFactory<flann::IndexParams, flann_algorithm_t>::instance().register_<flann::LinearIndexParams>(FLANN_INDEX_LINEAR);


    //ToDo make it flexible using aparam to indicate path to training data
    std::vector<vfh_model> models;
    flann::Matrix<float> data;

    std::string kdtree_idx_file_name = "/home/pinaki/hack-arena/ROS/extractObjects/kdtree.idx";
    std::string training_data_h5_file_name = "/home/pinaki/hack-arena/ROS/extractObjects/training_data.h5";
    std::string training_data_list_file_name = "/home/pinaki/hack-arena/ROS/extractObjects/training_data.list";

    // Check if the data has already been saved to disk

    if (!boost::filesystem::exists (training_data_h5_file_name) || !boost::filesystem::exists (training_data_list_file_name))
    {
        ROS_ERROR ("Could not find training data models files %s and %s!\n",
            training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
        return (0);
    }
      else
      {
        loadFileList (models, training_data_list_file_name);
        flann::load_from_file (data, training_data_h5_file_name, "training_data");
        ROS_INFO ("Training data found. Loaded %d VFH models from %s/%s.\n",
            (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
      }

      // Check if the tree index has already been saved to disk

      if (!boost::filesystem::exists (kdtree_idx_file_name))
      {
        ROS_ERROR ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
        exit(0);
      }

    flann::Index<flann::ChiSquareDistance<float> > tempIndex (data, flann::SavedIndexParams (kdtree_idx_file_name));
    tempIndex.buildIndex();



    vfhPoseEstimator poseEstimator;
    if( !poseEstimator.initialize( noOfNeighbors, distanceThreshold, &tempIndex, &models)) exit(0);

    ros::Subscriber  objectClusterSubscriber[noOfObjects];
    for (int i =0; i<noOfObjects; i++ ) {
      std::stringstream ss;
      ss << "object_cluster_" << i+1;
      objectClusterSubscriber[i]  = nh.subscribe(ss.str(), 1, &vfhPoseEstimator::objectClusterCallback, &poseEstimator);
    }


    ROS_INFO("Now estimating Object Poses ;)");

    string ch = "y";
    while( !ch.compare("y")) {

	ros::spinOnce();

    cout << "Do you want to continue ? "<< endl;
    cin >> ch;
    }
	return 0;

}
