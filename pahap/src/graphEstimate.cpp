#include "ros/ros.h"
// #include "pahap/pointcloud_cmd.h"
// #include "pahap/pointcloud_nav.h"
// #include "pahap/pcl_segment.h"
// #include "pahap/pcl_boundary.h"
// #include "pahap/displayData.h"

#include "geometry_msgs/Pose.h"
#include "pahap/cluster.h"
#include "pahap/graphEstimate.h"
#include <cstdlib>


namespace graphEstimate
{

// constructor
GraphEstimatE::GraphEstimatE(std::string name): 
  nhge_(ros::NodeHandle()),
  nhge_para_(ros::NodeHandle("~"))
  // pcl_input_(new pclXYZRGB),
  // pclFile_input_(new pclXYZRGB),
  // pclTopic_input_(new pclXYZRGB)
{
  // pclTopic_ = t_Name;
  // if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (f_Name, *pclFile_input_)) {
  //     PCL_ERROR ("Couldn't read the point cloud data file \n");
  // }
  // slicingFactor_ = s_Fact;
  // tolerance_ = tol;
  // showBound_ = s_Boun;
  // showSelRect_ = s_Rect;
  // showPose_ = s_Pose;
  // rFWidth_ = f_W;
  // rFLength_ = f_L;  
  // initialize other element
  // initPublisher();
  // initSubscriber();
  std::cout << "check the constructor:" << name << std::endl;
  initServer();
}

// void PahaP::initPublisher() {
//   // ros message publisher
//   pahap_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("processed_pcl", 10);
// }

// void PahaP::initSubscriber()
// {
//   // ros message subscriber
//   pahap_pclCam_sub_ = nh_.subscribe (pclTopic_, 1, &PahaP::pointcloudCallback, this);
// }

void GraphEstimatE::initServer() {
  pahap_geSer_ = nhge_.advertiseService("graph_estimate", &GraphEstimatE::graph_estimate, this);
  // std::cout << "can it go to here.    ......" << std::endl;
  // pahap_nav_ = nh_.advertiseService("pointcloud_nav", &PahaP::navigate, this);
  // pahap_bound_ = nh_.advertiseService("pcl_boundary", &PahaP::boundaryEstimate, this);
}

bool GraphEstimatE::graph_estimate(pahap::graph_estimate::Request &req, pahap::graph_estimate::Response &res) {
  if (req.cmd) {  
    // SvEv2f bound0, bound1, bound2, bound3, bound4, bound5, bound6, bound7, bound8, bound9;
    

    SvSvEv2f bounds;
    std::vector<Eigen::Vector2f> lineCooefs; //, lineCo1, lineCo2, lineCo3, lineCo4, lineCo5, lineCo6, lineCo7, lineCo8, lineCo9;
    SvEv4f lineBoun_inters; // first intersect = 0,1; second intersect = 2,3; 
    std::cout << req.clusterSet.size() << std::endl;
    for (int i =0; i < req.clusterSet.size(); i++) {
      Ev2f lineCo;
      Ev4f lineBoun_inter;
      SvEv2f bound;
      for (int j=0; j < req.numofData[i]; j++) {  
        Ev2f point;
        point(0) = req.clusterSet[i].cluster[j].x;
        point(1) = req.clusterSet[i].cluster[j].y;
        bound.push_back(point);
      }
      bounds.push_back(bound);
      lineCo = lineFit(bound);
      std::cout << lineCo << std::endl;


      lineCooefs.push_back(lineCo);
      lineBoun_inter = lineBound_intersect(lineCo, bound);
      lineBoun_inters.push_back(lineBoun_inter);
    }

    // std::cout << "can it go to here.    ......" << std::endl;

    // if (not (req.bound0.size() == 0)): {
    //   Ev2f point, lineCo;
    //   Ev4f lineBoun_inter;
    //   for (int i =0; i < req.bound0.size(); i++) {
    //     point(0) = req.bound0[i].x;
    //     point(1) = req.bound0[i].y;
    //     bound0.push_back(point);
    //     lineCo = lineFit(bound0);
    //     lineCooefs.push_back(lineCo);
    //     lineBoun_inter = lineBound_intersect(lineCo, bound0);
    //     lineBoun_inters.push_back(lineBoun_inter);
    //   } 
    // }
    // std::cout << lineCooefs[0] << std::endl;
    res.line_cooeficient.push_back(lineCooefs[0][0]);
    res.line_cooeficient.push_back(lineCooefs[0][1]);
    res.interPoints.push_back(lineBoun_inters[0][0]);
    res.interPoints.push_back(lineBoun_inters[0][1]);
    res.interPoints.push_back(lineBoun_inters[0][2]);
    res.interPoints.push_back(lineBoun_inters[0][3]);
    res.check = true;
    return true;
  }
  else return false;
}

// find a fit line by median
Ev2f GraphEstimatE::lineFit(SvEv2f bound){
  Ev2f cooef(0,0);
  float x_med, y_med, a, b;
  for (int i =0; i< bound.size(); i++) {
    x_med += bound[i][0];
    y_med += bound[i][1];
  }
  x_med = x_med/bound.size();
  y_med = y_med/bound.size();
  float nom, dem;
  for (int i =0; i< bound.size(); i++) {
    nom += (bound[i][0]-x_med)*(bound[i][1]-y_med);
    dem += (bound[i][0]-x_med)*(bound[i][0]-x_med);
  }
  a = nom/dem;
  b = y_med - a*x_med;
  cooef(0) = a;
  cooef(1) = b;
  return cooef;
}

Ev4f GraphEstimatE::lineBound_intersect(Ev2f lineCo, SvEv2f bound) {
  float distance =0, min_distance =0, check_distance;
  Ev4f interPoints;
  Ev2f point1, point2;
  for (int i =0; i < bound.size(); i++) {
    distance = abs(bound[i][1] - lineCo(0)*bound[i][0] - lineCo[1])/sqrt(pow(lineCo[0],2) +pow(lineCo[1],2));
    if (i==0) {
      min_distance = distance;
      point1 = bound[i];
    }
    if (min_distance > distance) {
      min_distance = distance;
      point1 = bound[i];
    }
  }
  min_distance = 2.0;
  // find the second intersection
  for (int i =0; i < bound.size(); i++) {
    // check with the first point - should be far way from it
    check_distance = sqrt(pow((bound[i][0] -point1[0]),2) + pow((bound[i][1] -point1[1]),2));
    if (check_distance > 0.1) {
      distance = abs(bound[i][1] - lineCo(0)*bound[i][0] - lineCo[1])/sqrt(pow(lineCo[0],2) +pow(lineCo[1],2));
      if (min_distance > distance) {
        min_distance = distance;
        point2 = bound[i];
      }
    }
  }
  interPoints(0) = point1(0);
  interPoints(1) = point1(1);
  interPoints(2) = point2(0);
  interPoints(3) = point2(1);
  return interPoints;
}



}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pahap_graphEstimation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_para("~");

  ros::Rate loop_rate(100);
  // std::cout << "just want to check where is the mistake??????" << std::endl;

  std::string addressToSave;
  nh_para.param("addressToSave", addressToSave, std::string("/home/buivn/bui_ws/src/pahap/pointcloud/"));  
  // bool fromTopic;
  // nh_para.param("fromTopic", fromTopic, false);
  
  // set for the service request
  // srv1.request.cmd = true;
  // srv1.request.path = addressToSave; 
  // srv1.request.topic = fromTopic;

  // int i = 0;
  // int numberData;
  graphEstimate::GraphEstimatE graph_estimate(addressToSave);

  ros::spin();
  return 0;
}