#!/usr/bin/env python
import rospy
import roslib
import sys
import numpy as np
from geometry_msgs.msg import Point
# from pahap.srv import image_cmd
from pahap.srv import pcl_segment, pcl_segmentResponse
from pahap.msg import cluster
# from pahap.srv import display2Ddata, display2DdataResponse

from numpy import expand_dims
import sys
from sklearn import mixture
import matplotlib.pyplot as plt


class dataSegment_server:
  def __init__(self):
    # self.image_topic = ""
    # self.image_file = ""
    # self.modelAdress = ""
    rospy.init_node('dataSegment')    # initialize a node
    

  def callSpawnServiceTopic(self):
    
    rospy.Service('pcl_segment', pcl_segment, self.dataSegment_response)   # advertise a service
    # rospy.Service('display_2D_data', display2Ddata, self.dataDisplay_response)   # advertise a service
    r= rospy.Rate(50)
    # print("Check check check .........")
    while not rospy.is_shutdown():
      r.sleep()

  def dataSegment_response(self, data):
    numofCluster = 0
    if data.cmd:
      X =[]
      Y =[]
      D =[]
      n = data.dataNumber
      # print('The number of data is: ',n)
      for i in range(n):
        # X.append(data.points[i].x)
        # Y.append(data.points[i].y)
        D.append([data.points[i].x, data.points[i].y])
      
      labels = []
      clusterSet=[]

      numofData = []
      opt_neigbor = 0.0
      opt_cluster = 0
      for i in range(3,7): # run from 3 -> 6
        # gaussian mixture model - maximum likelihood estimation
        em_gmm = mixture.GaussianMixture(n_components=i, covariance_type='full')
        # em_gmm = mixture.GaussianMixture(n_components=5, covariance_type='diag')
        # train model by fit function
        em_gmm.fit(D)
        # test model by predict function
        labels = em_gmm.predict(D)
        
        clusterSet1=[]
        boundarySet = []
        # push each labeled data into a set
        for k in range(i):
          points = [] 
          for j in range(n):
            if labels[j] == k:
              # point = Point()
              point_x = D[j][0]
              point_y = D[j][1]
              if len(points) ==0:
                points = np.array([[point_x, point_y]])
              else:
                points = np.append(points, [[point_x, point_y]], axis=0)

          clusterSet1.append(points)
        # print('number of cluster is hehre:  ', len(clusterSet1))

        for j in range(len(clusterSet1)):
          data1 = clusterSet1[j]
          min_max = self.det_maxmin_xy(data1)
          # print('check min max position: ', min_max)
          bound1 = self.est_2DBound(data1, min_max, 0.04)
          # print('check how much data in the boundary: ', bound1)
          boundarySet.append(bound1)
          # print('how much boundary do we have:', len(boundarySet))

        neighMatrix = self.determine_neighbors(boundarySet, 0.01)
        print(neighMatrix)
        max_neigh, second_neigh = self.max_clusterNeighbor(neighMatrix)

        print('the neighbor is: ', max_neigh)
        print('the cluster is: ', i)
        # ratio = (float(max_neigh) - float(second_neigh))/np.sqrt(float(i))
        ratio = float(max_neigh)/(float(max_neigh) + float(second_neigh)) + float(max_neigh)/np.sqrt(float(i))
        
        print('the neighbor ration is: ', ratio)
        if ratio > opt_neigbor:
          opt_neigbor = ratio
          opt_cluster = i


      # em_gmm = mixture.GaussianMixture(n_components=opt_cluster, covariance_type='full')
      em_gmm = mixture.GaussianMixture(n_components=3, covariance_type='full')
      # em_gmm = mixture.GaussianMixture(n_components=5, covariance_type='diag')
      # train model by fit function
      em_gmm.fit(D)
      # test model by predict function
      labels = em_gmm.predict(D)
      # print('the length of labels: ', len(labels))

      # data to show
      # get the segmented data and send back to the service
      for i in range(6):
        points = cluster()
        x1 =[]
        y1 = []
        for j in range(n):
          if labels[j] == i:
            point = Point()
            point.x = D[j][0]
            point.y = D[j][1]
            point.z = 0.0
            x1.append(D[j][0])
            y1.append(D[j][1])
            points.cluster.append(point)
            # print(point)
        if not (len(points.cluster) == 0):
          print(len(points.cluster))
          clusterSet.append(points)
          numofData.append(len(points.cluster))
          X.append(x1)
          Y.append(y1)

      # print (labels)
      # x0 =[]
      # y0 =[]
      # for j in range(len(d0)):
      #   x0.append(d0[j][0])
      #   y0.append(d0[j][1])
      plt.figure()
      # plt.subplot(221)
      color = ['r','b', 'y', 'm', 'g', 'c', 'k', 'w']
      for i in range(len(X)):
        plt.scatter(X[i], Y[i], c=color[i])
      # plt.subplot(222)
      # plt.scatter(x1, y1, c='b')
      # plt.subplot(223)
      # plt.scatter(x2, y2, c='g')      
      # plt.title("cluster 1")
      # plt.scatter(X, Y, c=labels, s=40, cmap='viridis')
      plt.title("EM-GMM")
      # plt.show()

      return pcl_segmentResponse(True, numofData, clusterSet)    # return the value for the service.response


  # get the maximum and minimum coordinate of 2D data
  def det_maxmin_xy(self, data): 
      max_min = np.array([0.0,0.0,0.0,0.0])
      max_x = 10.0
      min_x = -10.0
      max_y = 10.0
      min_y = -10.0
      for i in range(len(data)):    
        if i ==0: 
            max_x = data[i][0]
            min_x = data[i][0]
            max_y = data[i][1]
            min_y = data[i][1]
        
        # For x
        if data[i][0] > max_x:
            max_x = data[i][0]
        if data[i][0] < min_x:
            min_x = data[i][0]
        # For y
        if data[i][1] > max_y:
            max_y = data[i][1]
        if data[i][1] < min_y:
            min_y = data[i][1]
    
      max_min[0] = min_x;
      max_min[1] = max_x;
      max_min[2] = min_y;
      max_min[3] = max_y;
      return max_min
  

  # get the set of boundary points
  def est_2DBound(self, data, max_minxy, slicing_factor):  # slicing_factor =0.02
    twoDboundary = []
    point1 = np.array([0.0,0.0])
    point2 = np.array([0.0,0.0])
    start = 0.0
    distance1 = 0.0
    distance2 = 0.0
    max_distance = 0.0
    maxmin = np.array([0.0,0.0,0.0,0.0])
    for i in range(4):
        maxmin[i] = max_minxy[i]
    count_check = 0


    # slicing for each axis direction find the farthest point couples in each slicing axis
    for axis in range(2):
      start = maxmin[2*axis]
      # find the point cloud belong to one slicing
      while start < maxmin[2*axis+1]:
        for i in range(len(data)):
          # select the right distance
          dis_xy = 0.0
          if axis == 0:
            dis_xy = data[i][0] - start
          if axis == 1: 
            dis_xy = data[i][1] - start

          # if the point belong to the slicing, check whether it set a furthest distance pair
          test1 = dis_xy - slicing_factor/2
          test2 = dis_xy + slicing_factor/2

          # if (dis_xy < slicing_factor/2) and (dis_xy > -slicing_factor/2):
          if (test1 < 0) and (test2 > 0):
            if count_check == 0:  # this is the first point
              point1[0] = data[i][0]
              point1[1] = data[i][1]
            elif count_check == 1: # this is the second point
              point2[0] = data[i][0]
              point2[1] = data[i][1]
              max_distance = np.power((point2[0]-point1[0]),2) + np.power((point2[1]-point1[1]),2)
            else:
              distance1 = np.power((point1[0]-data[i][0]),2) + np.power((point1[1]-data[i][1]),2)
              distance2 = np.power((point2[0]-data[i][0]),2) + np.power((point2[1]-data[i][1]),2)
              if distance2 < distance1:
                if distance1 > max_distance: 
                  max_distance = distance1
                  point2[0] = data[i][0]
                  point2[1] = data[i][1]    
              if distance2 > distance1:
                if distance2 > max_distance: 
                  max_distance = distance2
                  point1[0] = data[i][0]
                  point1[1] = data[i][1]    
            count_check = count_check  + 1

        count_check = 0  
        # check if there is points with common coordinate
        commonCoor1 = False
        commonCoor2 = False
        if len(twoDboundary) == 0:
          twoDboundary = np.array([point1])
          twoDboundary = np.append(twoDboundary, [point2], axis=0)
        else:
          for j in range(len(twoDboundary)):
            if (point1[0] == twoDboundary[j][0]) and (point1[1] == twoDboundary[j][1]): 
              commonCoor1 = True
              # std::cout << "Point 1 has the same coordinate with a point in boundary set" << std::endl;
            if (point2[0] == twoDboundary[j][0]) and (point2[1] == twoDboundary[j][1]):
              commonCoor2 = True
              # std::cout << "Point 2 has the same coordinate with a point in boundary set" << std::endl;
          if (not commonCoor1):
            # print('how many time they will come herelsdfkjasldjfl;asjdl;jkfasl;df')
            # print('check the value of point 1', point1)
            twoDboundary = np.append(twoDboundary, [point1], axis=0)
          if (not commonCoor2):
            twoDboundary = np.append(twoDboundary, [point2], axis=0)


        # print('the length of the boundary set: ', len(twoDboundary))
        start = start + slicing_factor/2
        point1[0] = 0.0
        point1[1] = 0.0
        point2[0] = 0.0
        point2[1] = 0.0
    # twoDboundary = np.array(twoDboundary)
    return twoDboundary

  
  def determine_neighbors(self, data, min_dis): # data here is the boundarySet
    neig_matrix = np.zeros((len(data),len(data)))
    
    for i in range(len(data)-1):
      clus1 = data[i]
      for j in range(i+1,len(data)):
        borderPoints = []
        clus2 = data[j]
        for m in range(len(clus1)):
          for n in range(len(clus2)):
            distance = np.sqrt(np.power((clus1[m][0] -clus2[n][0]),2) + np.power((clus1[m][1] - clus2[n][1]),2))
            # if two points closer than the threshold
            if distance < min_dis:
              p_x = (clus1[m][0] + clus2[n][0])/2
              p_y = (clus1[m][1] + clus2[n][1])/2
              if len(borderPoints) == 0:
                borderPoints = np.array([[p_x, p_y]])
              else:
                borderPoints = np.append(borderPoints, [[p_x, p_y]], axis=0)
        
        # the border should be long enough to be considered as neighbors
        max_distance = 0.0
        if len(borderPoints) > 2:  
          for k in range(len(borderPoints)-1):
            for h in range(k, len(borderPoints)):
              dis = self.cal_distance(borderPoints[k], borderPoints[h])
              if max_distance < dis:
                max_distance = dis

        # condition to be neighbor is the border should longer than 0.1 
        if max_distance > 0.11:
          neig_matrix[i][j] = 1.0
          neig_matrix[j][i] = 1.0
        # print(neig_matrix)

    return neig_matrix

  def cal_distance(self, p1, p2):
    return np.sqrt(np.power((p1[0]-p2[0]),2) + np.power((p1[1]-p2[1]),2))

  def max_clusterNeighbor(self, neighMatrix):
    max_neighbor = 0.0
    sec_neighbor = 0.0
    cluster_index = 0
    for i in range(len(neighMatrix)):
      count =0
      for j in range(len(neighMatrix)):
        if neighMatrix[i][j] == 1.0:
          count += 1
      if count > max_neighbor:
        max_neighbor = count
        cluster_index = i
    # for the second most neighbor
    for i in range(len(neighMatrix)):
      count =0
      if not (i == cluster_index):
        for j in range(len(neighMatrix)):
          if neighMatrix[i][j] == 1.0:
            count += 1
        if count > sec_neighbor:
          sec_neighbor = count
          # cluster_index = i

    return max_neighbor, sec_neighbor




def main(args):
  
  segmentServer = dataSegment_server()
  segmentServer.callSpawnServiceTopic()

  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  # cv2.destroyAllWindows()



if __name__ == '__main__':

    main(sys.argv)
    # # topic name
    # pub_dzungYolov3Keras = rospy.Publisher('Object_Centroid', String, queue_size=2)
    # # node name
    # rospy.init_node('dzungyolov3keras', anonymous = True)
    # rate = rospy.Rate(0.2)
