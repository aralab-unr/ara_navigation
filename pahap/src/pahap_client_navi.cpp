#include "ros/ros.h"
#include "pahap/pointcloud_cmd.h"
#include "pahap/pointcloud_nav.h"
#include "pahap/pcl_segment.h"
#include "pahap/pcl_boundary.h"
#include "pahap/displayData.h"
#include "pahap/graph_estimate.h"
#include "pahap/cluster.h"
#include "pahap/rrt.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pahap_navigation");
  ros::NodeHandle n;
  ros::NodeHandle n_para("~");
  ros::ServiceClient client1 = n.serviceClient<pahap::pointcloud_nav>("pointcloud_nav");
  ros::ServiceClient client2 = n.serviceClient<pahap::pcl_segment>("pcl_segment");
  ros::ServiceClient client3 = n.serviceClient<pahap::pcl_boundary>("pcl_boundary");
  
  ros::ServiceClient client4 = n.serviceClient<pahap::displayData>("displayData");
  
  ros::ServiceClient client5 = n.serviceClient<pahap::graph_estimate>("graph_estimate");
  ros::ServiceClient client6 = n.serviceClient<pahap::rrt>("rrt_motionPlanning");

  pahap::pointcloud_nav srv1;
  pahap::pcl_segment srv2;
  pahap::pcl_boundary srv3;
  pahap::displayData srv4;
  pahap::graph_estimate srv5;
  pahap::rrt srv6;

  ros::Rate loop_rate(0.2);

  std::string addressToSave;
  n_para.param("addressToSave", addressToSave, std::string("/home/buivn/bui_ws/src/pahap/pointcloud/"));  
  bool fromTopic;
  n_para.param("fromTopic", fromTopic, false);
  
  // set for the service request
  srv1.request.cmd = true;
  srv1.request.path = addressToSave; 
  srv1.request.topic = fromTopic;

  int i = 0;
  int numberData;
  while(ros::ok()) {
    if (i<1) {
      srv1.request.num_name = std::to_string(i);
      if (client1.call(srv1)) {
        ROS_INFO("Service 1 - projected data - the code is running well");
        numberData = srv1.response.dataNumber;
        for (int j =0; j < numberData; j++){
          // prepare to transfer the data to segmentation function
          srv2.request.points.push_back(srv1.response.points[j]);
          // ROS_INFO("Position.x = ");
        }
        srv2.request.dataNumber = numberData;
        srv2.request.cmd = true;
      }

      else {
        ROS_ERROR(" Service 1 - There is something wrong: ");
      }
      
      if (client2.call(srv2)) {
        if (srv2.response.test) {
          ROS_INFO("Service 2 - The segmentation code runs well");
          for (int i = 0; i < srv2.response.clusterSet.size(); i++) {
            pahap::cluster clus;
            for (int j = 0; j < srv2.response.numofData[i]; j++) {
              clus.cluster.push_back(srv2.response.clusterSet[i].cluster[j]);
              // std::cout << srv2.response.cluster1[i] << std::endl;
            }
            srv3.request.clusterSet.push_back(clus);
            srv3.request.numofData.push_back(srv2.response.numofData[i]);
          }
          srv3.request.cmd = true;
        }
        else {
          ROS_ERROR(" Service 2 - There is something wrong: ");
        }
      }

      if (client3.call(srv3)) {
        if (srv3.response.test) {
          ROS_INFO("Service 3 - The boundary estimation code runs well");
          for (int j =0; j < srv3.response.clusterSet.size(); j++){
            // prepare to transfer the data to segmentation function          
            srv5.request.clusterSet.push_back(srv3.response.clusterSet[j]);
            srv5.request.numofData.push_back(srv3.response.clusterSet[j].cluster.size());

            srv6.request.clusterSet.push_back(srv3.response.clusterSet[j]);
            // srv6.request.numofData.push_back(srv3.response.clusterSet[j].cluster.size());
            
          }
          srv5.request.cmd = true;    
        }
      }
      else {
        ROS_ERROR(" Service 3 - There is something wrong: ");
      }
      

      if (client5.call(srv5)) {
        if (srv5.response.check) {
          ROS_INFO("Service 5 - Got the line, graph, and intersection point");

          for (int i =0; i < srv5.response.vocpp_path.size(); i++)
            std::cout << srv5.response.vocpp_path[i] << std::endl;
          srv6.request.vocpp_path = srv5.response.vocpp_path;
          srv6.request.cmd = true;
          // for (int i =0; i < srv5.response.interPoints.size(); i++)
            // srv4.request.intersections.push_back(srv5.response.interPoints[i]);
        }
      }
      else {
        ROS_ERROR(" Service 5 - There is something wrong: ");
      }

      if (client6.call(srv6)) {
        if (srv6.response.check) {
          ROS_INFO("Service 6 - motion planning");
        }
      }
      else {
        ROS_ERROR(" Service 6 - Motion Planning - There is something wrong: ");
      }
    }
    i += 1;
    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}