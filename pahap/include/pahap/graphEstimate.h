#ifndef GRAPHESTIMATE_H_
#define GRAPHESTIMATE_H_

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "geometry_msgs/Pose.h"
#include "pahap/cluster.h"
#include "pahap/graph_estimate.h"

#include <vector>

// using EvXf = Eigen::VectorXf;
using Ev4f = Eigen::Vector4f;
// using Ev3f = Eigen::Vector3f;
using Ev2f = Eigen::Vector2f;
// using EmXf = Eigen::MatrixXf;
// using Em4f = Eigen::Matrix4f;
// using SvEv3f = std::vector<Eigen::Vector3f>;
using SvEv2f = std::vector<Eigen::Vector2f>;
using SvEv4f = std::vector<Eigen::Vector4f>;
using SvSvEv2f = std::vector<std::vector<Eigen::Vector2f>>;

// struct projectedPoint {
//       float x;
//       float y;
// };

namespace graphEstimate
{
  class GraphEstimatE
  {
    public:     // public function
      // constructor with a point cloud topic
      GraphEstimatE(std::string name);
      // constructor with a pcd file
      // PahaP(std::string file_name, int check);
      // ~PahaP();
        // Define a data structure: Points


      // list of displaying functions
      // void display_pcd(const pclXYZRGB::Ptr cloud, int clus_order, const std::string& titlename);
      // void display_centroid_multCoor(pclXYZRGB::Ptr cloud, EmXf& centroid3D);

      

      // initialize function
      // void initPublisher();
      // void initSubscriber();
      void initServer();
     
      // for service 3
      // EvXf det_maxmin_xy(SvEv3f data);
      // SvEv2f est_2DBound(SvEv3f data, EvXf max_minxy, float slicing_factor);

      Ev2f lineFit(SvEv2f bound);
      Ev4f lineBound_intersect(Ev2f lineCo, SvEv2f bound);

      // list of Callbackfunction
      // void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
      // some services
      // bool check_and_save(pahap::pointcloud_cmd::Request &req, pahap::pointcloud_cmd::Response &res);
      // bool navigate(pahap::pointcloud_nav::Request &req, pahap::pointcloud_nav::Response &res);
      // bool boundaryEstimate(pahap::pcl_boundary::Request &req, pahap::pcl_boundary::Response &res);
      bool graph_estimate(pahap::graph_estimate::Request &req, pahap::graph_estimate::Response &res);


    private:
      
      ros::NodeHandle nhge_; // general ROS NodeHandle - used for pub, sub, advertise ...
      ros::NodeHandle nhge_para_; // private ROS NodeHandle - used to get parameter from server
      // // ROS Publisher
      // ros::Publisher pahap_pose_pub_;
      // ros::Publisher pahap_pcl_pub_;
      
      // // ROS Subscribers
      // ros::Subscriber pahap_pclCam_sub_;
      
      // ROS Service Server
      ros::ServiceServer pahap_geSer_;
      // ros::ServiceServer pahap_nav_;
      // ros::ServiceServer pahap_bound_;

      // // Topic name
      // std::string pclTopic_;
      // // Create a container for the input point cloud data.
      // pclXYZRGB::Ptr pclTopic_input_;
      // pclXYZRGB::Ptr pclFile_input_;
      // pclXYZRGB::Ptr pcl_input_;

      // double slicingFactor_;
      // double tolerance_;
      // double rFWidth_;
      // double rFLength_;
      // bool showBound_;
      // bool showSelRect_;
      // bool showPose_;
  };

}
#endif /* GRAPHESTIMATE_ */
