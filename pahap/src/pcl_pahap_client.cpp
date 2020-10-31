#include "ros/ros.h"
#include "pahap/pointcloud_cmd.h"
#include "geometry_msgs/Pose.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_pahap_client");
  ros::NodeHandle n;
  ros::NodeHandle n_para("~");
  ros::ServiceClient client = n.serviceClient<pahap::pointcloud_cmd>("pointcloud_cmd");
  pahap::pointcloud_cmd srv;
  ros::Rate loop_rate(0.2);

  std::string addressToSave;
  n_para.param("addressToSave", addressToSave, std::string("/home/buivn/bui_ws/src/pahap/pointcloud/"));  
  bool requestTopic;
  n_para.param("requestTopic", requestTopic, true);
  
  // set for the service request
  srv.request.cmd = true;
  srv.request.path = addressToSave; 
  srv.request.topic = requestTopic;

  int i = 0;
  while(ros::ok()) {
    if (i<2) {
      srv.request.num_name = std::to_string(i);
      client.call(srv);
      // if (client.call(srv))
      if ((srv.response.planePose.position.x == 0) and (srv.response.planePose.position.y == 0) and (srv.response.planePose.position.z == 0)) {
        ROS_INFO("The is no pose detected");
      }
      else {
        ROS_INFO("One Pose is detected: ");
        ROS_INFO("Position: x = [%f]", srv.response.planePose.position.x);
        ROS_INFO("Position: y = [%f]", srv.response.planePose.position.y);
        ROS_INFO("Position: z = [%f]", srv.response.planePose.position.z);
        ROS_INFO("Orientation: w = [%f]", srv.response.planePose.orientation.w);
        ROS_INFO("Orientation: x = [%f]", srv.response.planePose.orientation.x);
        ROS_INFO("Orientation: y = [%f]", srv.response.planePose.orientation.y);
        ROS_INFO("Orientation: z = [%f]", srv.response.planePose.orientation.z);
        // return 1;
      }
      // else ROS_ERROR("Fail to connect Service pointcloud_cmd");
    }
    i += 1;
    ros::spinOnce();
    loop_rate.sleep();
  } 


  return 0;
}