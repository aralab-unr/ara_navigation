#!/usr/bin/env python
import rospy
import roslib
import sys
import numpy as np
from geometry_msgs.msg import Point
# from pahap.srv import image_cmd
# from pahap.srv import pcl_segment, pcl_segmentResponse
from pahap.srv import displayData, displayDataResponse

from numpy import expand_dims
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
# import cv2
# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
from sklearn import mixture
import matplotlib.pyplot as plt


class dataSegment_server:
  def __init__(self):
    # self.image_topic = ""
    # self.image_file = ""
    # self.modelAdress = ""
    rospy.init_node('dataDisplay')    # initialize a node
    

  def callSpawnServiceTopic(self):
    
    # rospy.Service('pcl_segment', pcl_segment, self.dataSegment_response)   # advertise a service
    rospy.Service('displayData', displayData, self.dataDisplay_response)   # advertise a service
    r= rospy.Rate(50)
    # print("Check check check .........")
    while not rospy.is_shutdown():
      r.sleep()
 
  def dataDisplay_response(self, data):
    if data.cmd:
      X =[]
      Y =[]
      # D =[]
      # print('Check2 check2 check2 .........')
      n = len(data.points)
      print('The number of data is: ',n)
      x_min = 0
      x_max = 0
      for i in range(n):
        X.append(data.points[i].x)
        Y.append(data.points[i].y)
        if i == 0:
          x_min = data.points[i].x
          x_max = data.points[i].x
        if data.points[i].x > x_max:
          x_max = data.points[i].x
        if data.points[i].x < x_min:
          x_min = data.points[i].x



      if not (len(data.line_cooef) ==0): 
        print(data.line_cooef)
        a = data.line_cooef[0]
        b = data.line_cooef[1]

      # x range
      x_axis = np.linspace(x_min, x_max)

      # compute the y for each x using the polynomial
      y_axis = a*x_axis + b

      

      plt.figure()
      plt.plot(x_axis, y_axis, label='fit-line')
      plt.scatter(X, Y, c='r')
          
      plt.title("Boundary and fit line")      
      plt.show()

      return displayDataResponse(True)    # return the value for the service.response   

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
