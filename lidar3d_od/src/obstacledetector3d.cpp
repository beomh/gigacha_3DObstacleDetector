#include "processPointClouds.h"
#include "processPointClouds.cpp"
// PCL specific includes

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
// ConstPtr
// void MakeVisual(const sensor_msgs::PointCloud2& cloud_msg, int num){
// 	sensor_msgs::PointCloud2 input_cloud;
//   input_cloud = cloud_msg;
//   sensor_msgs::PointCloud out_cloud;
//   sensor_msgs::convertPointCloud2ToPointCloud(input_cloud, out_cloud);

//   float avr_x, avr_y, avr_z;
//   float min_x = out_cloud.points[0].x;
//   float min_y = out_cloud.points[0].y;
//   float min_z = out_cloud.points[0].z;

//   float max_x = out_cloud.points[0].x;
//   float max_y = out_cloud.points[0].y;
//   float max_z = out_cloud.points[0].z;

//   for (int i = 0; i < out_cloud.points.size(); i++) {
//     if (out_cloud.points[i].x > max_x) max_x = out_cloud.points[i].x;
//     if (out_cloud.points[i].x < min_x) min_x = out_cloud.points[i].x;
//     if (out_cloud.points[i].y > max_y) max_y = out_cloud.points[i].y;
//     if (out_cloud.points[i].y < min_y) min_y = out_cloud.points[i].y;
//     if (out_cloud.points[i].z > max_z) max_z = out_cloud.points[i].z;
//     if (out_cloud.points[i].z < min_z) min_z = out_cloud.points[i].z;
//   }

//   // std::cout<<"max x : " << max_x<<std::endl;
//   // std::cout<<"min x : " << min_x<<std::endl;
//   visualization_msgs::MarkerArray markers_array;
//   visualization_msgs::Marker obs_marker;

//   obs_marker.header.frame_id = "velodyne";
//   obs_marker.header.stamp = ros::Time::now();
//   obs_marker.ns = "name_space";
//   obs_marker.id = num;
//   obs_marker.type = visualization_msgs::Marker::CUBE;
//   obs_marker.action = visualization_msgs::Marker::ADD;

//   obs_marker.pose.position.x = (max_x + min_x) / 2;
//   obs_marker.pose.position.y = (max_y + min_y) / 2;
//   obs_marker.pose.position.z = (max_z + min_z) / 2;

//   obs_marker.pose.orientation.x = 0.0;
//   obs_marker.pose.orientation.y = 0.0;
//   obs_marker.pose.orientation.z = 0.0;
//   obs_marker.pose.orientation.w = 1.0;

//   obs_marker.scale.x = fabs(max_x - min_x);
//   obs_marker.scale.y = fabs(max_y - min_y);
//   obs_marker.scale.z = fabs(max_z - min_z);

//   srand((unsigned int)time(NULL));

//   // int colorarr[3];

//   // for (int i = 0; i < 3; ++i) {
//   //       int num = rand();
//   //       colorarr[i] =  (int)num % 100;
//   // }

//   obs_marker.color.r = 255;
//   obs_marker.color.g = 0;
//   obs_marker.color.b = 0;
//   obs_marker.color.a = 0.7;

//   obs_marker.lifetime = ros::Duration(0.1);

//   markers_array.markers.push_back(obs_marker);

//   pub3.publish(markers_array);
// }

void cloud_cb (const sensor_msgs::PointCloud2& ros_pc)
{
    pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
    pcl_conversions::toPCL(ros_pc, pcl_pc);
    // Create output point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_ptr(new pcl::PointCloud<pcl::PointXYZI>());

    // Convert point cloud to PCL native point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    cerr << "Raw " << input_ptr->points.size() << "\n";

    /*Filter: Crop the scene to requested dimensions and remove the points from the roof top*/
	  output_ptr = pointProcessorI->FilterCloud(input_ptr, 0.13); // or *input_ptr
    //pointProcessorI->numPoints(output_ptr);
    /*Segmentation: Segment the scene to create to clouds one for road and another for objects.*/
	  //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane_RANSAC(output_ptr, 50, 0.2);
    /*Clustering: Identify the clusters from objects*/
	  //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering_euclideanCluster(segmentCloud.first, 0.5, 100, 1000);
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 20, 1500);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(output_ptr, 0.6, 10, 1500);

    int clusterId = 0;
    visualization_msgs::MarkerArray markers_array;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
      std::cout << "cluster size ";
      pointProcessorI->numPoints(cluster);

      pcl::PCLPointCloud2 pcl_pc1; // temporary PointCloud2 intermediary
      sensor_msgs::PointCloud2 cluster_output;
      pcl::toPCLPointCloud2(*cluster, pcl_pc1);
      pcl_conversions::fromPCL(pcl_pc1, cluster_output);
      cluster_output.header.frame_id = "velodyne";

      sensor_msgs::PointCloud2 input_cloud;
      input_cloud = cluster_output;
      sensor_msgs::PointCloud out_cloud;
      sensor_msgs::convertPointCloud2ToPointCloud(input_cloud, out_cloud);

      float avr_x, avr_y, avr_z;
      float min_x = out_cloud.points[0].x;
      float min_y = out_cloud.points[0].y;
      float min_z = out_cloud.points[0].z;

      float max_x = out_cloud.points[0].x;
      float max_y = out_cloud.points[0].y;
      float max_z = out_cloud.points[0].z;

      for (int i = 0; i < out_cloud.points.size(); i++) {
        if (out_cloud.points[i].x > max_x) max_x = out_cloud.points[i].x;
        if (out_cloud.points[i].x < min_x) min_x = out_cloud.points[i].x;
        if (out_cloud.points[i].y > max_y) max_y = out_cloud.points[i].y;
        if (out_cloud.points[i].y < min_y) min_y = out_cloud.points[i].y;
        if (out_cloud.points[i].z > max_z) max_z = out_cloud.points[i].z;
        if (out_cloud.points[i].z < min_z) min_z = out_cloud.points[i].z;
      }

      // visualization_msgs::MarkerArray markers_array;
      visualization_msgs::Marker obs_marker;

      obs_marker.header.frame_id = "velodyne";
      obs_marker.header.stamp = ros::Time::now();
      obs_marker.ns = "name_space";
      obs_marker.id = clusterId;
      obs_marker.type = visualization_msgs::Marker::CUBE;
      obs_marker.action = visualization_msgs::Marker::ADD;

      obs_marker.pose.position.x = (max_x + min_x) / 2;
      obs_marker.pose.position.y = (max_y + min_y) / 2;
      obs_marker.pose.position.z = (max_z + min_z) / 2;

      obs_marker.pose.orientation.x = 0.0;
      obs_marker.pose.orientation.y = 0.0;
      obs_marker.pose.orientation.z = 0.0;
      obs_marker.pose.orientation.w = 1.0;

      obs_marker.scale.x = fabs(max_x - min_x);
      obs_marker.scale.y = fabs(max_y - min_y);
      obs_marker.scale.z = fabs(max_z - min_z);

      srand((unsigned int)time(NULL));

      obs_marker.color.r = 255;
      obs_marker.color.g = 0;
      obs_marker.color.b = 0;
      obs_marker.color.a = 0.7;

      obs_marker.lifetime = ros::Duration(0.1);

      markers_array.markers.push_back(obs_marker);
      
      ++clusterId;
    }

    // Now covert output back from PCL native type to ROS
    sensor_msgs::PointCloud2 ros_output;
    pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, ros_output);
    // std::cout << ros_output << std::endl;

    // Publish the data
    pub1.publish(ros_output);
    pub3.publish(markers_array);
}

int main(int argc, char** argv)
{
  cout << "Obstacle Detection ON\n";
  // Initialize ROS
  ros::init (argc, argv, "ObstacleDetector3d");
  ros::NodeHandle nh;

  // // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  //pub2 = nh.advertise<sensor_msgs::PointCloud2> ("clusters", 1);
  pub3 = nh.advertise<visualization_msgs::MarkerArray>("obstacles_markers", 10);

  // Spin
  ros::spin ();
}