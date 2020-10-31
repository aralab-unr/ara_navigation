#ifndef PAHAP_H_
#define PAHAP_H_

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
#include "ros/ros.h"
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/don.h>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include "geometry_msgs/Pose.h"
#include <pcl_ros/impl/transforms.hpp>

#include "pahap/pointcloud_cmd.h"
#include "pahap/pointcloud_nav.h"
#include "pahap/pcl_boundary.h"
#include "pahap/cluster.h"

// #include "pcl_segment.h"
#include <vector>

using pclXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using pclXYZ = pcl::PointCloud<pcl::PointXYZ>;
using pclNormal = pcl::PointCloud<pcl::PointNormal>;
using SvPclIndices = std::vector<pcl::PointIndices>;
using EvXf = Eigen::VectorXf;
using Ev4f = Eigen::Vector4f;
using Ev3f = Eigen::Vector3f;
using EmXf = Eigen::MatrixXf;
using Em4f = Eigen::Matrix4f;
using SvEv3f = std::vector<Eigen::Vector3f>;
using SvEv2f = std::vector<Eigen::Vector2f>;

struct projectedPoint {
      float x;
      float y;
};

namespace pahap
{
  class PahaP
  {
    public:     // public function
      // constructor with a point cloud topic
      PahaP(std::string t_Name, std::string f_Name, double s_Fact, double tol, bool s_Boun, bool s_Rect, bool s_Pose, double f_W, double f_L);
      // constructor with a pcd file
      // PahaP(std::string file_name, int check);
      // ~PahaP();
        // Define a data structure: Points


      // list of displaying functions
      void display_pcd(const pclXYZRGB::Ptr cloud, int clus_order, const std::string& titlename);
      void display_centroid_multCoor(pclXYZRGB::Ptr cloud, EmXf& centroid3D);
      void display_normals(const pclXYZRGB::Ptr cloudp, const pclNormal::Ptr normals);
      void display_planeNormals(pclXYZRGB::Ptr cloud, Ev3f normals, Ev4f centroid);
      void display_clofarPair(pclXYZRGB::Ptr cloud, EvXf closfart, Ev4f centroid);
      void display_targetCoordinate(pclXYZRGB::Ptr cloud, Em4f RotMatrix);
      void display_pointSet_asPointCloud(SvEv3f setOfPoint);
      void draw_Rectangles(EmXf fiRect, pclXYZRGB::Ptr cloudp);
      

      // initialize function
      void initPublisher();
      void initSubscriber();
      void initServer();

      // list of propressing functions
      pclXYZRGB pcd_read(const std::string& filename);
      pclXYZRGB::Ptr passthrough_filter(pclXYZRGB::Ptr cloud); // passThrough filter
      pclXYZRGB::Ptr downsampling(pclXYZRGB::Ptr cloud);
      pclXYZRGB::Ptr est_plane(pclXYZRGB::Ptr cloud);
      
      // new for the navigate service
      pclXYZRGB::Ptr est_multPlane(pclXYZRGB::Ptr cloud);
      pclXYZRGB::Ptr pcl_frameTransform(pclXYZRGB::Ptr cloud);
      Em4f eulerAngletoRotationMatrix(float x, float y, float z, float roll, float pitch, float yaw);
      std::vector<projectedPoint> project_pclData(pclXYZRGB::Ptr cloud);
      
      // for service 3
      EvXf det_maxmin_xy(SvEv3f data);
      SvEv2f est_2DBound(SvEv3f data, EvXf max_minxy, float slicing_factor);


      Ev3f est_planeNormal(pclXYZRGB::Ptr cloud);  // estimate the normal vector of a plane
      EvXf det_MaxMinxyz(pclXYZRGB::Ptr cloudp);  // det max & min coordinate of point cloud according to xyz
      SvEv3f est_BounPoint(pclXYZRGB::Ptr cloudp, EvXf MaxMinxyz,float slicing_factor); // est the boundary point set
      // estimate the closest and farthest points to the centroid
      EvXf est_clofarPoints_toCentroid(SvEv3f B_Point, Ev4f centroid);
      float cal_PlaneArea(const EvXf clos_fart);  // approximate the area by closest and farthest point pair
      // calculate the Quaternions with the closest -farthest point pair
      geometry_msgs::Pose est_Pose1(Ev4f centroid, EvXf clos_fart, Ev3f ave_normal);
      Em4f convert_PosetoRotationMatrix(geometry_msgs::Pose robot_pose);       // convert Quaternion to heterousgeneous matrix
      EmXf est_5closPoints_toCentroid(SvEv3f B_Point, Ev4f centroid);
      // determine 5 rectangles with five closest point to the centroid
      EmXf det_5rectangle(EmXf fiPoint, Ev4f centroid, Ev3f normal, float l, float w);
      EmXf select_Rectangle(EmXf fiRect, Ev4f centroid, SvEv3f B_Point, double tolerance);
      geometry_msgs::Pose est_Pose2(EmXf rectangle, Ev3f ave_normal); // estimate the Quaternions by five closest points
      SvPclIndices extract_objects(pclXYZRGB::Ptr cloud);
      Em4f est_objectCentroids(pclXYZRGB::Ptr cloud, SvPclIndices c_indices, int nCluster);
      pclXYZRGB::Ptr integrate_clusters(pclXYZRGB::Ptr cloud, SvPclIndices cluster_indices);

      void pcd_write_unPcl(const std::string& filename, pclXYZRGB::Ptr cloud); // this one save unorganined point cloud
      
      // pointcloud publish function
      void pointCloudCb(const pclXYZRGB::Ptr cloud);
      

      // list of Callbackfunction
      void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
      // some services
      bool check_and_save(pahap::pointcloud_cmd::Request &req, pahap::pointcloud_cmd::Response &res);
      bool navigate(pahap::pointcloud_nav::Request &req, pahap::pointcloud_nav::Response &res);
      bool boundaryEstimate(pahap::pcl_boundary::Request &req, pahap::pcl_boundary::Response &res);

    private:
      
      ros::NodeHandle nh_; // general ROS NodeHandle - used for pub, sub, advertise ...
      ros::NodeHandle nh_private_; // private ROS NodeHandle - used to get parameter from server
      // ROS Publisher
      ros::Publisher pahap_pose_pub_;
      ros::Publisher pahap_pcl_pub_;
      
      // ROS Subscribers
      ros::Subscriber pahap_pclCam_sub_;
      // ROS Service Server
      ros::ServiceServer pahap_getPcl_ser_;
      ros::ServiceServer pahap_nav_;
      ros::ServiceServer pahap_bound_;

      // Topic name
      std::string pclTopic_;
      // Create a container for the input point cloud data.
      pclXYZRGB::Ptr pclTopic_input_;
      pclXYZRGB::Ptr pclFile_input_;
      pclXYZRGB::Ptr pcl_input_;

      double slicingFactor_;
      double tolerance_;
      double rFWidth_;
      double rFLength_;
      bool showBound_;
      bool showSelRect_;
      bool showPose_;
  };

}
#endif /* PAHAP_ */
