// ROS, system library
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <vector>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// Self-defined msg
#include <kinect_data/CloudSegmentedArray.h>
#include <kinect_data/Location.h>
#include <kinect_data/LocationArray.h>


// helper function
// Estimate bounding rectangle in XY plane
float* vec_min_range(std::vector<float> v);
// distribute RGB color for visualization
int* acquire_rgb (int co);

// callback function
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

// publisher
ros::Publisher pub, pub_vis;

// Global setting
bool visualization = true;
float downsample_size = 0.015;
float z_filter_thres[2] = {0, 3.5};
float y_filter_thres[2] = {-.8, 5};
int iter_ransac_plan_removal = 130;
float dist_plan_removal = 0.04;
int euclidean_cluster_min = 300;
int euclidean_cluster_max = 10000;
float euclidean_cluster_tolerance = 0.035; //cm


// MAIN
int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "kinect_data");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output cloud data
  ros::NodeHandle ntest;
  pub_vis = ntest.advertise<sensor_msgs::PointCloud2> ("kinect_data/visualization", 1);
  pub = nh.advertise<kinect_data::LocationArray> ("kinect_data/obstacles_location", 1);

  // Spin
  ros::spin ();
}


// Main process
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  
  /*
  0, Transfer from sensor PC2 to pcl PC2
  */
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  /*
  0.5, ----TEST ONLY---- 
  //Transfer from pcl XYZRGB to sensor PC2
  // declare the output variable instances
  pcl::PCLPointCloud2 pcl_PC2;
  sensor_msgs::PointCloud2 output;
  // convert
  pcl::toPCLPointCloud2( *clusterPtr, pcl_PC2);
  pcl_conversions::fromPCL(pcl_PC2, output);
  //add in set
  CloudClusters.clusters.push_back(output);
  pub_vis.publish(output);
  */
  
  /*
  1, perform a voxel grid downsamplng to reduce 
     the pointclouds number
  */
  // Perform voxel grid downsampling filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (downsample_size, downsample_size, downsample_size);
  sor.filter (cloud_filtered);
  

  /*
  1.5, Transfer from pcl PC2 to pcl XYZRGB
  */
  // Container for pcl XYZRGB data
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr(xyz_cloud); 
  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(cloud_filtered, *xyzCloudPtr);
  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  /*
  2, perform a pass through filter to get rid of
     pointclouds far from camera
  */
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  // Create the filtering object for z(depth)
  pass.setInputCloud (xyzCloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_filter_thres[0], z_filter_thres[1]);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);
  // Create the filtering object for y(height)
  pass.setInputCloud (xyzCloudPtrFiltered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_filter_thres[0], y_filter_thres[1]);
  pass.filter (*xyzCloudPtrFiltered);


  /*
  3, perform a plane detection to remove points of 
     horizontal plane and width plane
  */
  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

  // perform ransac planar filtration to remove table top
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations(iter_ransac_plan_removal);
  seg.setDistanceThreshold (dist_plan_removal);
  // Filter Direction
  // Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
  // seg.setAxis(axis);
  // seg.setEpsAngle(  5.0f * (M_PI/180.0f) );
  seg.setInputCloud (xyzCloudPtrFiltered);
  seg.segment (*inliers, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);


  /*
  4, perform euclidean cluster segmentation to seporate individual objects
     Create the KdTree object for the search method of the extraction
  */
  // create a tree for cluster
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance (euclidean_cluster_tolerance); // 3.5cm
  ec.setMinClusterSize (euclidean_cluster_min);
  ec.setMaxClusterSize (euclidean_cluster_max);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  ec.extract (cluster_indices);


  ////(1/3)DEMONSTRATION of cluster, used when only demonstration==true 
  //*TIME CONSUMING
  pcl::PointCloud<pcl::PointXYZRGB> *demo = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr demoPtr (demo);
 
  // declare an output for data
  kinect_data::LocationArray Obstacles;   

  // object number counter
  int co = 0;
  // object points containers
  std::vector<float> points_X;
  std::vector<float> points_Y;
  std::vector<float> points_Z;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    // create a pcl object to hold the extracted cluster
    pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

    // now we are in a vector of indices per
taining to a single cluster.
    // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered 
    // a specific color for identification purposes
    
    // container for determine obstacle's location
    kinect_data::Location obstacle;
    
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);
      points_X.push_back(xyzCloudPtrRansacFiltered->points[*pit].x);
      points_Y.push_back(xyzCloudPtrRansacFiltered->points[*pit].y);
      points_Z.push_back(xyzCloudPtrRansacFiltered->points[*pit].z);
      
      if (visualization == true) {
        ////(2/3)DEMONSTRATION color setting // not well defined :(
        demoPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);
        int* rgb;
        rgb = acquire_rgb(co);
        demoPtr->points.back().r = rgb[0];
        demoPtr->points.back().g = rgb[1];
        demoPtr->points.back().b = rgb[2];
      }
    }
    co++;
    
    // Estimate bouding rectangle of object
    float* min_range_x; 
    float* min_range_z; 
    min_range_x = vec_min_range(points_X);
    min_range_z = vec_min_range(points_Z);
    obstacle.x = min_range_x[0];
    obstacle.y = min_range_z[0];
    obstacle.width = min_range_x[1];
    obstacle.height = min_range_z[1];
 
    // Clear for the next object
    points_X.clear();
    points_Y.clear();
    points_Z.clear();

    // add this object to object array
    Obstacles.locations.push_back(obstacle);
  }
  Obstacles.length = co;


  /*
  5, publish result
  */
  // Publish the data.
  pub.publish(Obstacles);
  if (visualization == true) {
    ////(3/3)DEMONSTRATION the result
    demoPtr->width = demoPtr->points.size ();
    demoPtr->height = 1;
    demoPtr->is_dense = true;
    demoPtr->header = xyzCloudPtrRansacFiltered->header;
    pcl::PCLPointCloud2 pcl_PC2;
    sensor_msgs::PointCloud2 output;
    pcl::toPCLPointCloud2(*demoPtr, pcl_PC2);
    pcl_conversions::fromPCL(pcl_PC2, output);
    pub_vis.publish (output);
  }
}


// Compute the min and range of a vector
float* vec_min_range (std::vector<float> v) {
  // define a container for min and range
  float *result = new float;
  float max = -1000;
  float min = 1000;

  for (int i = 0; i < v.size(); ++i){
    if (v[i] < min) 
      min = v[i];
    if (v[i] > max) 
      max = v[i];
  }

  result[0] = min;
  result[1] = max - min;
  return result;
}


// How boring I am to write a function to deal with color =.=
int* acquire_rgb (int co){
  int* rgb = new int;
  int r, g, b;
  if (co < 4) {
    r = 60*co;
    g = 25*co;
    b = 16*co;
  }
  else if (co < 8){
    r = 255-16*co;
    g = 25*co;
    b = 16*co;
  }
  else if (co < 12){
    r = 255-16*co;
    g = 255-25*co;
    b = 16*co;
  }
  else{
    r = 255-16*co;
    g = 255-25*co;
    b = 255-60*co;
  }
  rgb[0] = r;
  rgb[1] = g;
  rgb[2] = b;
  return rgb;
}
