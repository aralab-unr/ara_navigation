#!/usr/bin/env python
import rospy
import roslib
import sys
import numpy as np
import random
from geometry_msgs.msg import Point
from pahap.msg import cluster
from pahap.srv import graph_estimate, graph_estimateResponse

from numpy import expand_dims
import sys
from sklearn import mixture
import matplotlib.pyplot as plt
import numpy.linalg as linalg
import copy
from chinesepostman import eularian, network



class graphEstimate_server:
  def __init__(self):
    # self.image_topic = ""
    # self.image_file = ""
    # self.modelAdress = ""
    rospy.init_node('graph_estimate')    # initialize a node
    

  def callSpawnServiceTopic(self):
    
    # rospy.Service('pcl_segment', pcl_segment, self.dataSegment_response)   # advertise a service
    rospy.Service('graph_estimate', graph_estimate, self.graphEstimate_response)   # advertise a service
    r= rospy.Rate(50)
    # print("Check check check .........")
    while not rospy.is_shutdown():
      r.sleep()

  def graphEstimate_response(self, data):
    if data.cmd:
      X =[]
      Y =[]
      line_cooefs = []
      # bounInterSects = []
      clusterSet = []
      centers = []

      for j in range(len(data.clusterSet)):
        edges = []
        vertices = []
        
        x = []
        y = []
        D = np.array([[0,0]])
        for i in range(len(data.clusterSet[j].cluster)):
          x.append(data.clusterSet[j].cluster[i].x)
          y.append(data.clusterSet[j].cluster[i].y)
          insert1 = np.array([[data.clusterSet[j].cluster[i].x, data.clusterSet[j].cluster[i].y]])
          if i == 0:
            D[0,0] = insert1[0,0]
            D[0,1] = insert1[0,1]
          else: 
            D = np.append(D, insert1, axis=0)
        X.append(x)
        Y.append(y)
        # center = self.deter_centerPoint(D)
        center = np.mean(D, axis=0)
        if len(centers) ==0:
          centers = np.array([center])
        else: 
          centers = np.append(centers, [center], axis=0)
        # determine the feature lines
        line_cooef = self.lineFit_pca(D)
        line_cooefs.append(line_cooef)

        clusterSet.append(D)

      
      # find all the neighbor cluster
      neigh_matrix = self.find_neighClus(clusterSet, 0.008)
      borPoint_matrix = self.find_midBorPoints(neigh_matrix, clusterSet)

      g_ver, g_edges = self.gCon_midBorder(centers, borPoint_matrix)
      # print('check the edges current form')
      # print(g_edges)
      mod_edges = []
      # convert the edge to the right form for vocpp
      for i in range(len(g_edges)):
        mod_edges.append((int(g_edges[i][0]), int(g_edges[i][1]), round(g_edges[i][2],4)))
      print(mod_edges)

      original_graph = network.Graph(mod_edges)
      print(original_graph.nodes)
      
      # find the starting point and target point
      d_min = 0.0
      d_max = 0.0
      v_dmin = 0
      v_dmax = 0
      for i in range(len(g_ver)):
        dis = self.dis_twoPoints([0,0], g_ver[i])
        if i == 0:
          d_min = dis
          v_dmin = i
          d_max = dis
          v_dmax = i
        else:
          if d_min > dis:
            d_min = dis
            v_dmin = i
          if d_max < dis:
            d_max = dis
            v_dmax = i
            
      graph = eularian.make_vocpp(original_graph, v_dmin, v_dmax)
      # print(graph.odd_nodes)
      route, attempts = eularian.eularian_path(graph, start=v_dmin)
      print('\t{}'.format(route))
      print(route)

      # prepare data to response
      
      points = []
      for i in range(len(route)):
        point = Point()
        point.x = g_ver[i][0]
        point.y = g_ver[i][1]
        point.z = 0.0
        points.append(point)

      # g_ver1 = self.graph_filter(g_ver, g_edges)

      for k in range(len(route)-1):
        v_x2 = g_ver[route[k]][0]
        v_y2 = g_ver[route[k]][1]
        v_x3 = g_ver[route[k+1]][0]
        v_y3 = g_ver[route[k+1]][1]
        del_x = 0.7*(v_x3 - v_x2)
        del_y = 0.7*(v_y3 - v_y2)
        add = random.uniform(-1.0,1.0)/55
        plt.arrow(v_x2+add, v_y2+add, del_x, del_y, head_width=0.025, width=0.003)

      v_x = []
      v_y= []
      # for k in range(len(g_ver)):
      for k in range(len(centers)):
        v_x.append(g_ver[k][0])
        v_y.append(g_ver[k][1])
      v_x1 = []
      v_y1= []
      # for k in range(len(g_ver)):
      for k in range(len(centers), len(g_ver)):
        v_x1.append(g_ver[k][0])
        v_y1.append(g_ver[k][1])

      # v_x2 = []
      # v_y2= []

      # for k in range(len(g_edges)):
      #   v_x2 = g_ver[int(g_edges[k][0])][0]
      #   v_y2 = g_ver[int(g_edges[k][0])][1]
      #   v_x3 = g_ver[int(g_edges[k][1])][0]
      #   v_y3 = g_ver[int(g_edges[k][1])][1]
      #   plt.plot([v_x2, v_x3], [v_y2, v_y3], 'k-')
        

      for i in range(len(X)):
        # if i ==0:
        # plt.plot(x_axis[i], y_axis[i], label='fit-line')
        color = ['b','r', 'y', 'm', 'g', 'c', 'k', 'w']
        plt.scatter(X[i], Y[i], c=color[i])
        # plt.ylim(-0.35, 0.35)
        # plt.title("Boundary and fit line")  
      
      plt.scatter(v_x, v_y, c='k')  
      plt.scatter(v_x1, v_y1, c='c')  

      # plt.show()      


      # for k in range(len(neighNumber)):
      return graph_estimateResponse(True, points, None, None, None)      



 
  def graphEstimate_response2(self, data):
    if data.cmd:
      X =[]
      Y =[]
      # print('The number of data is: ',n)
      x_minset = []
      x_maxset = []
      line_cooefs = []
      bounInterSects = []
      clusterSet = []

      # some new ideas
      clustersEdgeVertices_set = []

      for j in range(len(data.clusterSet)):
        edges = []
        vertices = []
        x = []
        y = []
        D = np.array([[0,0]])
        for i in range(len(data.clusterSet[j].cluster)):
          x.append(data.clusterSet[j].cluster[i].x)
          y.append(data.clusterSet[j].cluster[i].y)
          insert1 = np.array([[data.clusterSet[j].cluster[i].x, data.clusterSet[j].cluster[i].y]])
          if i == 0:
            D[0,0] = insert1[0,0]
            D[0,1] = insert1[0,1]
          else: 
            D = np.append(D, insert1, axis=0)
        X.append(x)
        Y.append(y)
        # center = self.deter_centerPoint(D)
        center = np.mean(D, axis=0)
        # add this center point to the first vertex inthe vertices set of this cluster
        vertices = np.array([center])
        # determine the feature lines
        line_cooef = self.lineFit_pca(D)
        line_cooefs.append(line_cooef)
        # find the intersection between the feature lines and the boundary
        interSect = self.lineBound_intersect(line_cooef, D)
        vertices = np.append(vertices, [[interSect[0], interSect[1]]], axis=0)
        vertices = np.append(vertices, [[interSect[2], interSect[3]]], axis=0)
        # add the edges
        dis1 = self.dis_twoPoints(center, [interSect[0], interSect[1]])
        dis2 = self.dis_twoPoints(center, [interSect[2], interSect[3]])
        edge = np.array([0, 1, dis1])
        edges = np.array([edge])
        edge = np.array([0, 2, dis2])
        edges = np.append(edges, [edge], axis=0)
        clustersEdgeVertices_set.append([vertices, edges])
        clusterSet.append(D)

      # find all the neighbor cluster
      neigh_matrix = self.find_neighClus(clusterSet, 0.008)
      # print(neigh_matrix)
      # set an order to process from the most neighbor clusters
      neighNumber = []
      for i in range(len(neigh_matrix)):
        mostNeigh = 0.0
        for j in range(len(neigh_matrix)):
          mostNeigh = mostNeigh + neigh_matrix[i][j]
          # print( 'to check', mostNeigh)
        if len(neighNumber) ==0:
          neighNumber = np.array([[i, mostNeigh]]) 
        else:
          neighNumber = np.append(neighNumber, [[i, mostNeigh]], axis=0)
      # print(neighNumber)
      # sorting decreasing order
      
      neighNumber = neighNumber[neighNumber[:,1].argsort()]
      neighNumber = np.flip(neighNumber, axis=0)
      # print(neighNumber)

      # consider only the cross area
      vertices_cross = []   # vertices set of cross area and its neighbor
      edges_cross = []  # edges set of cross area and its neighbor
      neighborEdgeVertices_set = []
      firstNeigh = 0
      for h in range(len(neigh_matrix[int(neighNumber[0][0])])):
        # if the clusters are neighbors
        if neigh_matrix[int(neighNumber[0][0])][h] == 1.0:
          data1 = clusterSet[int(neighNumber[0][0])]
          data2 = clusterSet[h]
          # check how close feature lines and center points
          cross_edgeVertex = clustersEdgeVertices_set[int(neighNumber[0][0])]
          neigh_edgeVertex = clustersEdgeVertices_set[h]
          # if the distance between the center of cross area with feature line of its neighbor
          if firstNeigh ==0:
            vertices_cross.append(cross_edgeVertex[0][0])
            firstNeigh = 1
          # print(vertices_cross[0])
          # print(line_cooefs[h])
          if self.point_lineDistance(vertices_cross[0], line_cooefs[h]) < 0.08:
            dis1 = self.dis_twoPoints(vertices_cross[0], neigh_edgeVertex[0][1])
            dis2 = self.dis_twoPoints(vertices_cross[0], neigh_edgeVertex[0][2])
            if dis1 < dis2:
              vertices_cross.append(neigh_edgeVertex[0][1])
              edges_cross.append([0, len(vertices_cross)-1, dis1])
            else:
              vertices_cross.append(neigh_edgeVertex[0][2])
              edges_cross.append([0, len(vertices_cross)-1, dis2])
          
          else: # the feature line does not cross the center point
            # get the intersection point
            inSePoint = self.twolines_intersect(line_cooefs[int(neighNumber[0][0])], line_cooefs[h])
            # check which cluster boundary the point belong
            insideCheck1 = self.check_inside(inSePoint, clusterSet[int(neighNumber[0][0])])
            insideCheck2 = self.check_inside(inSePoint, clusterSet[h])


            # if the point belong to cross area
            if insideCheck1:
              # find closer edge point
              getdata = clustersEdgeVertices_set[int(neighNumber[0][0])]
              d_min = 5.0
              edP_index = 0
              for i in range(len(getdata[0])):
                if i>0: # not check with the center
                  dis3 = self.dis_twoPoints(inSePoint, getdata[0][i])
                  if d_min < dis3:
                    d_min = dis3
                    edP_index = i
              clustersEdgeVertices_set[int(neighNumber[0][0])][0]= np.append(clustersEdgeVertices_set[int(neighNumber[0][0])][0], [inSePoint], axis=0)
              # adjust one edge
              for f in range(len(getdata[1])):
                if clustersEdgeVertices_set[int(neighNumber[0][0])][1][f][1] == edP_index:
                  clustersEdgeVertices_set[int(neighNumber[0][0])][1][f][1] = len(clustersEdgeVertices_set[int(neighNumber[0][0])][0])-1
                  dis3 = self.dis_twoPoints(inSePoint, getdata[0][0])
                  clustersEdgeVertices_set[int(neighNumber[0][0])][1][f][2] = dis3
              # add new edge
              clustersEdgeVertices_set[int(neighNumber[0][0])][1] = np.append(clustersEdgeVertices_set[int(neighNumber[0][0])][1], [[i, len(clustersEdgeVertices_set[int(neighNumber[0][0])][0])-1, d_min]], axis=0)

              # make a new edge for the neighborVerticeEdge
              #add to this set vertices_cross, edges_cross
              vertices_cross.append(inSePoint)
              getdata = clustersEdgeVertices_set[h]
              dis4 = self.dis_twoPoints(inSePoint, getdata[0][1])
              dis5 = self.dis_twoPoints(inSePoint, getdata[0][2])
              if dis4 < dis5:
                vertices_cross.append(getdata[0][1])
                edges_cross.append([len(vertices_cross)-2,len(vertices_cross)-1, dis4])
              else:
                vertices_cross.append(getdata[0][2])
                # add edge
                edges_cross.append([len(vertices_cross)-2,len(vertices_cross)-1, dis5])

            #if point belong to the neighbor boundary
            if insideCheck2:
              getdata = clustersEdgeVertices_set[h]
              dis4 = self.dis_twoPoints(inSePoint, getdata[0][0])
              if dis4 < 0.1:
                vertices_cross.append(getdata[0][0])
              else:
                vertices_cross.append(inSePoint)
                ########################## could need more code here
              # check all vertices of cross area
              getdata = clustersEdgeVertices_set[neighNumber[0][0]]
              d_min = 0.0
              vertex_index = 0
              for f in range(len(getdata[0])):
                dis5 = self.dis_twoPoints(inSePoint, getdata[0][f])
                if f == 0:
                  d_min = dis5
                if d_min > dis5:
                  d_min = dis5
                  vertex_index = f
              # add the next vertex
              vertices_cross.append(getdata[0][f])
              dis5 = self.dis_twoPoints(vertices_cross[len(vertices_cross)-2], vertices_cross[len(vertices_cross)-1])
              edges_cross.append([vertices_cross[len(vertices_cross)-2],vertices_cross[len(vertices_cross)-1], dis5])



              # check whether is is close the center point
              # if yes, add a edge to connect the center point and the closest edge point of cross area
                    # mark not consider this neighborhood any more
          neigh_matrix[int(neighNumber[0][0])][h] = 0.0
      # add all vertices and edge of cross area and its neighbor to the neighborEdgeVertices_set
      
      neighborEdgeVertices_set.append([vertices_cross, edges_cross])

      g_vertices, g_edges = self.graph_construct_new(clustersEdgeVertices_set, neighborEdgeVertices_set)
      # filter_ver = self.graph_filter(g_vertices, g_edges)

      v_x = []
      v_y= []
      for k in range(len(g_vertices)):
        v_x.append(g_vertices[k][0])
        v_y.append(g_vertices[k][1])

      for i in range(len(X)):
        # if i ==0:
        # plt.plot(x_axis[i], y_axis[i], label='fit-line')
        color = ['b','r', 'y', 'm', 'g', 'c', 'k', 'w']
        plt.scatter(X[i], Y[i], c=color[i])
        # plt.ylim(-0.35, 0.35)
        # plt.title("Boundary and fit line")  
      
      plt.scatter(v_x, v_y, c='k')  

      plt.show()      


      # for k in range(len(neighNumber)):



      return graph_estimateResponse(True, None, None, None, None)    # return the value for the service.response   

 
  def graphEstimate_response1(self, data):
    if data.cmd:
      X =[]
      Y =[]
      # print('The number of data is: ',n)
      x_minset = []
      x_maxset = []
      line_cooefs = []
      bounInterSects = []
      clusterSet = []

      for j in range(len(data.clusterSet)):
        # n = data.numofData[j]
        D = np.array([[0,0]])
        x =[]
        y = []
        x_min = 0
        x_max = 0
        for i in range(len(data.clusterSet[j].cluster)):
          x.append(data.clusterSet[j].cluster[i].x)
          y.append(data.clusterSet[j].cluster[i].y)
          insert1 = np.array([[data.clusterSet[j].cluster[i].x, data.clusterSet[j].cluster[i].y]])
          if i == 0:
            D[0,0] = insert1[0,0]
            D[0,1] = insert1[0,1]
          else: 
            D = np.append(D, insert1, axis=0)
          if (i == 0):
            x_min = data.clusterSet[j].cluster[i].x
            x_max = data.clusterSet[j].cluster[i].x
          if data.clusterSet[j].cluster[i].x > x_max:
            x_max = data.clusterSet[j].cluster[i].x
          if data.clusterSet[j].cluster[i].x < x_min:
            x_min = data.clusterSet[j].cluster[i].x
          # print(i) 
        x_minset.append(x_min)
        x_maxset.append(x_max)
        X.append(x)
        Y.append(y)
        clusterSet.append(D)

        line_cooef = self.lineFit_pca(D)
        line_cooefs.append(line_cooef)
        interSect = self.lineBound_intersect(line_cooef, D)
        # print(interSect)
        if (j==0):
          bounInterSects = np.array([interSect])
        else:
          bounInterSects = np.append(bounInterSects, [interSect], axis=0)
      # convert to np array
      line_cooefs = np.array(line_cooefs)
      # x range
      x_axis = []
      y_axis = []
      # # compute the y for each x using the polynomial
      for i in range(len(line_cooefs)):
        x = np.linspace(x_minset[i], x_maxset[i])
        x_axis.append(x)
      for i in range(len(line_cooefs)):
        y = line_cooefs[i,0]*x_axis[i] + line_cooefs[i,1]
        y_axis.append(y)


      # find the line intersections
      # lineInters = self.lines_intersect(line_cooefs);
      # find the neighbor cluster 
      neigh_matrix = self.find_neighClus(clusterSet);
      # print(neigh_matrix)

      vertices, edges = self.graph_construct(clusterSet, neigh_matrix, line_cooefs, bounInterSects)
      print(vertices)
      v_x = []
      v_y = []
      for i in range(len(vertices)):
        v_x.append(vertices[i,0])
        v_y.append(vertices[i,1])

      plt.figure()
      for i in range(len(X)):
        # if i ==0:
        # plt.plot(x_axis[i], y_axis[i], label='fit-line')
        color = ['b','r', 'y', 'm', 'g', 'c', 'k', 'w']
        plt.scatter(X[i], Y[i], c=color[i])
        plt.ylim(-0.35, 0.35)
        plt.title("Boundary and fit line")  
        plt.show()

      
      # for i in range(len(vertices)):
        # plt.scatter(v_x[i], v_y[i], c='g', s=200)  


      # for i in range(len(bounInterSects)):
        # plt.scatter(bounInterSects[i,0], bounInterSects[i,1], c='b')
        # plt.scatter(bounInterSects[i,2], bounInterSects[i,3], c='b')
      # for i in range(len(lineInters)):
        # plt.scatter(lineInters[i,0], lineInters[i,1], c='y')
      # plt.ylim(-0.35, 0.35)
      # plt.title("Boundary and fit line")      
      # plt.show()

      return graph_estimateResponse(True, None, None, None, None)    # return the value for the service.response   

  def gCon_midBorder(self, centers, borPoint_matrix):
    vertices = copy.deepcopy(centers)
    # print('check check')
    # print(vertices)
    edges = []
    for i in range(len(borPoint_matrix)-1):
      for j in range(i+1,len(borPoint_matrix)):
        if (borPoint_matrix[i][j][0] ==0) and (borPoint_matrix[i][j][1] ==0):
          pass
        else:
          vertices = np.append(vertices, [borPoint_matrix[i][j]], axis=0)
    # print(vertices)

    for g in range(len(centers)):
      for h in range(len(borPoint_matrix)):
        if (borPoint_matrix[g][h][0] ==0) and (borPoint_matrix[g][h][1] ==0):
          pass
        else:
          # find the vertex-index in the vertices set 
          for k in range(len(vertices)):
            if (borPoint_matrix[g][h][0]==vertices[k][0]) and (borPoint_matrix[g][h][1]==vertices[k][1]):
              vertex_index = k
              dis = self.dis_twoPoints(centers[g], borPoint_matrix[i][j])
              if len(edges) ==0:
                edges = np.array([[g,vertex_index, dis]])
              else:
                edges = np.append(edges, [[g,vertex_index, dis]], axis=0)

    return vertices, edges




  def find_midBorPoints(self,neigh_matrix, clusterSet):
    midBorPoints = np.zeros((len(neigh_matrix),len(neigh_matrix),2))
    for i in range(len(clusterSet)-1):
      for j in range(i+1,len(clusterSet)):
        if neigh_matrix[i][j] == 1.0:
          borPoints = self.border_determine(clusterSet[i], clusterSet[j], 0.01)
          # print(borPoints)
          middleBorPoint = np.mean(borPoints, axis=0)
          midBorPoints[i][j][0] = middleBorPoint[0]
          midBorPoints[i][j][1] = middleBorPoint[1]
          midBorPoints[j][i][0] = middleBorPoint[0]
          midBorPoints[j][i][1] = middleBorPoint[1]
    return midBorPoints



  def graph_filter(self, verList, edList):
    # merge all close vertices into a node
    # find a list of close vertices  
    mod_verList = copy.deepcopy(verList)
    mod_edList = copy.deepcopy(edList)
    loop = 3
    for x in range(3):
      closeVers =[]
      condition = False
      # print(len(verList))
      # boolean index for each vertex 
      vertex_index = np.zeros((len(mod_verList),1))
      for i in range(len(mod_verList)-1):
        for j in range(i+1, len(mod_verList)):
          if (vertex_index[i] == 0.0) and (vertex_index[j] ==0.0):
            dis = self.dis_twoPoints(mod_verList[i], mod_verList[j])
            if dis < 0.08:
              condition = True
              vertex_index[i] = 0.0
              vertex_index[j] ==0.0
              if len(closeVers) ==0:
                closeVers = np.array([[i,j]])
              else:
                closeVers = np.append(closeVers, [[i,j]], axis=0)
      newVerList = []
      
      if condition:
        for k in range(len(mod_verList)): #-len(closeVers)):
          if (mod_verList[k][0] == 10.0) and (mod_verList[k][1] == 10.0):
            pass
          else:
            for h in range(len(closeVers)):
              if closeVers[h][0] == k:
                v_x = (mod_verList[k][0] + mod_verList[closeVers[h][1]][0])/2
                v_y = (mod_verList[k][1] + mod_verList[closeVers[h][1]][1])/2
                mod_verList[k][0] = 10.0  # not consider this vertex any more
                mod_verList[k][1] = 10.0
                mod_verList[closeVers[h][0]][0] = 10.0 # not consider this vertex any more
                mod_verList[closeVers[h][0]][1] = 10.0
                newVerList.append([v_x, v_y])
                for i in range(len(mod_edList)):
                  if (mod_edList[i][0]==closeVers[h][1] ) or (mod_edList[i][0]==closeVers[h][1]):
                    mod_edList[i][0] = len(newVerList)-1
                  if (mod_edList[i][1]==closeVers[h][1] ) or (mod_edList[i][1]==closeVers[h][1]):
                    mod_edList[i][1] = len(newVerList)-1

              elif closeVers[h][1] == k:
                v_x = (mod_verList[k][0] + mod_verList[closeVers[h][0]][0])/2
                v_y = (mod_verList[k][1] + mod_verList[closeVers[h][0]][1])/2
                newVerList.append([v_x, v_y])
                mod_verList[k][0] = 10.0 # not consider this vertex any more
                mod_verList[k][1] = 10.0
                mod_verList[closeVers[h][0]][0] = 10.0 # not consider this vertex any more
                mod_verList[closeVers[h][0]][1] = 10.0
                for i in range(len(mod_edList)):
                  if (mod_edList[i][0]==closeVers[h][1] ) or (mod_edList[i][0]==closeVers[h][1]):
                    mod_edList[i][0] = len(newVerList)-1
                  if (mod_edList[i][1]==closeVers[h][1] ) or (mod_edList[i][1]==closeVers[h][1]):
                    mod_edList[i][1] = len(newVerList)-1
              else:
                newVerList.append([mod_verList[k][0],mod_verList[k][0]])
                for i in range(len(mod_edList)):
                  if mod_edList[i][0]==k:
                    mod_edList[i][0] = len(newVerList)-1
                  if mod_edList[i][1]==k:
                    mod_edList[i][1] = len(newVerList)-1
        

        newVerList = np.array(newVerList)
        print('check the newVerList:-------')
        print(newVerList)
        # mod_verList = np.zeros((len(newVerList),1))
        print ('check the mod_verList before copy')
        print(mod_verList)
        mod_verList = np.empty([len(newVerList),2])
        print ('check the mod_verList after copy')
        print(mod_verList)
        
    return True
  

  def graph_construct_new(self, InEdVer_set, neighEdVer_set):
    vertice_list = []
    edge_list = []
    # first, just add the Inside Vertices and Edges
    for i in range(len(InEdVer_set)):
      data = InEdVer_set[i]
      verLen = len(vertice_list)
      # add vertices
      for j in range(len(data[0])):
        if verLen == 0:
          # print('check the value of adding: ', data[0][j])
          vertice_list = np.array([data[0][j]])
        else:
          vertice_list = np.append(vertice_list, [data[0][j]], axis=0)
      # add edge
      for k in range(len(data[1])):
        newVerIndex1 = data[1][k][0] + verLen
        newVerIndex2 = data[1][k][1] + verLen
        if len(edge_list) ==0:
          edge_list = np.array([[newVerIndex1, newVerIndex2, data[1][k][2]]])
        else: 
          edge_list = np.append(edge_list, [[newVerIndex1, newVerIndex2, data[1][k][2]]], axis=0)
    # print('The length of vertice_list in step 1: ', len(vertice_list))
    # print('check the value of vertice_list: ', vertice_list)

    # second, just add the Vertices and Edges from the neighbors
    # for h in range(len(neighEdVer_set)):
    #   data = neighEdVer_set[h]
    #   for l in range(len(data[0])):
    #     for m in range(len(vertice_list)):
    #       # print(vertice_list[m])
    #       # print(data[0][l][0])
    #       if (vertice_list[m][0]==data[0][l][0]) and (vertice_list[m][1]==data[0][l][1]):
    #         pass
    #       else:
    #         vertice_list = np.append(vertice_list, [data[0][l]], axis=0)
    #   v_index1 = 0
    #   v_index2 = 0
    #   for n in range(len(data[1])):
    #     for g in range(len(vertice_list)):
    #       if (vertice_list[g][0]==data[0][data[1][n][0]][0]) and (vertice_list[m][1]==data[0][data[1][n][0]][1]):
    #         v_index1 = g
    #       if (vertice_list[g][0]==data[0][data[1][n][1]][0]) and (vertice_list[m][1]==data[0][data[1][n][1]][1]):
    #         v_index2 = g

    #     edge_list = np.append(edge_list, [[v_index1, v_index2, data[1][n][2]]], axis=0)
    return vertice_list, edge_list


 
  def check_inside(self, node, data): # data = boundary. node = point
      cen = np.mean(data, axis=0)
      cen_x = cen[0]
      cen_y = cen[1]
      # distance from center point to node
      dcn = self.dis_twoPoints(node, [cen_x, cen_y])
      # find four closest points to the node - point
      # data2 = data[clus_index[j]]
      neigh_index = np.array([0,0,0,0])
      dist4neighbors = [] #np.array([1.0,1.0,1.0,1.0])
      max_distance = 0.0
      point_index = 0
      closetPoints = []
      # find 4 closest point around the robot point in each cluster
      for k in range(len(data)):
        dis = self.dis_twoPoints(node, data[k])
        if k < 4:
          dist4neighbors.append(dis)
          if k ==0:
            closetPoints = np.array([data[k]])
          else:
            closetPoints = np.append(closetPoints, [data[k]], axis=0)
          if dis > max_distance:
            max_distance = dis
            point_index = k   
        elif dis < max_distance:
          max_distance = dis
          dist4neighbors[point_index] = dis
          closetPoints[point_index][0] = data[k][0]
          closetPoints[point_index][1] = data[k][1]
          for m in range(len(dist4neighbors)):
            if max_distance < dist4neighbors[m]:
              max_distance = dist4neighbors[m]
              point_index = m

      d_nc = [] 
      for g in range(len(dist4neighbors)):
        d_cn = self.dis_twoPoints(closetPoints[g], [cen_x, cen_y])
        d_nc.append(d_cn)
      
      # boolean check set
      bcs = [False, False, False, False]
      for h in range(len(d_nc)):
          if (dcn - d_nc[h]) < 0.006:
              bcs[h] = True
      check = True
      for u in range(len(bcs)):
          check = check and bcs[u]
      if check:
          return True # Inside 
      return False  # outside


  def point_lineDistance(self, p1, line):
    # print('inside the point_lineDistance function')
    # print(p1)
    # print(line)
    nom = np.absolute(p1[1] - line[0]*p1[0] - line[1])
    dem = np.sqrt(np.power(line[0],2) + np.power(line[1],2))
    return nom/dem

  def deter_centerPoint(self, data):
    x = 0.0
    y = 0.0
    for i in range(len(data)):
      x += data[i][0]
      y += data[i][1]
    x = x/len(data)
    y = y/len(data)
    return np.array([x,y])

  
  def lineFit_pca(self, X):
    mean = np.mean(X, axis=0)
    mean_x = mean[0]
    mean_y = mean[1]
    mean_sub_matrix = np.zeros(X.shape)
    for i in range(len(mean_sub_matrix)):
      mean_sub_matrix[i,0] = X[i,0] - mean_x
      mean_sub_matrix[i,1] = X[i,1] - mean_y

    # calculate the covariance matrix
    cov_matr = np.matmul(mean_sub_matrix.transpose(),mean_sub_matrix)
    
    eigenValues, eigenVectors = linalg.eig(cov_matr)
    idx = eigenValues.argsort()[::-1]
    eigenValues = eigenValues[idx]
    eigenVectors = eigenVectors[:,idx]

    if eigenValues[0] > eigenValues[1]:
      m = eigenVectors[0,0]
      n = eigenVectors[0,1]
    else:
      m = eigenVectors[1,0]
      n = eigenVectors[1,1]
    a = n/m
    b = (m*mean_y - n*mean_x)/m
    line_cooef = np.array([a,b])
    return line_cooef

  def lineBound_intersect(self, lineCo, bound): 
    distance =0.0
    min_distance =0.0
    check_distance=0.0
    interPoints = []
    point1 = []
    point2 = []
    # convert a list to an array
    for i in range(len(bound)):
      distance = np.absolute(bound[i,1]-lineCo[0]*bound[i,0]-lineCo[1])/np.sqrt(np.power(lineCo[0],2)+np.power(lineCo[1],2))
      if (i==0) :
        min_distance = distance;
        point1 = bound[i]
      
      if (min_distance > distance): 
        min_distance = distance
        point1 = np.array([bound[i][0],bound[i][1]])
      
    min_distance = 2.0
    # find the second intersection
    for i in range(len(bound)):
      # check with the first point - should be far way from it
      check_distance = np.sqrt(np.power((bound[i,0] -point1[0]),2) + np.power((bound[i,1] -point1[1]),2))
      if (check_distance > 0.1):
        distance = np.absolute(bound[i,1] - lineCo[0]*bound[i,0]-lineCo[1])/np.sqrt(np.power(lineCo[0],2) +np.power(lineCo[1],2))
        if (min_distance > distance):
          min_distance = distance
          point2 = np.array([bound[i][0],bound[i][1]])     
      
    interPoints = np.array([point1[0], point1[1], point2[0], point2[1]])
    return interPoints

  def lines_intersect(self, lineCos):
    lineInterPoints = []
    count = 0
    for i in range(len(lineCos)-1):
      for j in range(i+1, len(lineCos)):
        dem = lineCos[i,0] - lineCos[j,0]
        # check whether two lines are parallel
        if np.absolute(dem) > 0.05:
          nom_x = lineCos[j,1] - lineCos[i,1]
          nom_y = lineCos[i,0]*lineCos[j,1] - lineCos[j,0]*lineCos[i,1]
          interSect_x = nom_x/dem
          interSect_y = nom_y/dem
          if count == 0:
            lineInterPoints = np.array([[interSect_x, interSect_y]])
          else:
            lineInterPoints = np.append(lineInterPoints, [[interSect_x,interSect_y]], axis=0)
          count += 1;
    # print ("the number of intersection: ", len(lineInterPoints))

    return lineInterPoints

  def twolines_intersect(self, lineCo1, lineCo2):
    lineInterPoint = []
    dem = lineCo1[0] - lineCo2[0]
    if np.absolute(dem) > 0.02:
      nom_x = lineCo2[1] - lineCo1[1]
      nom_y = lineCo1[0]*lineCo2[1] - lineCo2[0]*lineCo1[1]
      interSect_x = nom_x/dem
      interSect_y = nom_y/dem
      lineInterPoint = np.array([interSect_x, interSect_y])
    # print ("the number of intersection: ", len(lineInterPoints))
    return lineInterPoint

  def border_determine(self, bound1, bound2, min_dis):
    borderPoints = []
    # count1 = 0
    for m in range(len(bound1)):
      count2 = 0
      for n in range(len(bound2)):
        if count2 == 0:
          distance = 10.0
          distance = self.dis_twoPoints(bound1[m], bound2[n])

          # if two points closer than the threshold
          if distance < min_dis:
            # print('check the distance', distance)
            p_x = (bound1[m][0] + bound2[n][0])/2

            p_y = (bound1[m][1] + bound2[n][1])/2
            # print ('the two points: ', p_x, p_y)
            if len(borderPoints) == 0:
              borderPoints = np.array([[p_x, p_y]])
              # print('check the first point')
              # print(borderPoints)
              # print('check:', p_x, p_y)
            else:
              borderPoints = np.append(borderPoints, [[p_x, p_y]], axis=0)
            count2 = 1
    # remove the first point due to noise
    if not (len(borderPoints) == 0):
      borderPoints = np.delete(borderPoints, 0, 0)
    return borderPoints
        
  def find_neighClus(self, data, min_dis):
    neig_matrix = np.zeros((len(data),len(data)))
    
    for i in range(len(data)-1):
      clus1 = data[i]
      for j in range(i+1,len(data)):
        borderPoints = []
        clus2 = data[j]
        borderPoints = self.border_determine(clus1, clus2, min_dis)        
        # the border should be long enough to be considered as neighbors
        max_distance = 0.0
        if len(borderPoints) > 2:  
          for k in range(len(borderPoints)-1):
            for h in range(k+1, len(borderPoints)):
              dis = self.dis_twoPoints(borderPoints[k], borderPoints[h])
              if max_distance < dis:
                max_distance = dis

        # condition to be neighbor is the border should longer than 0.1 
        if max_distance > 0.08:
          neig_matrix[i][j] = 1.0
          neig_matrix[j][i] = 1.0
        # print(neig_matrix)

    return neig_matrix

  def find_neighClus1(self, data):
    neig_matrix = np.zeros((len(data),len(data)))

    for i in range(len(data)-1):
      # clus1 = []
      clus1 = data[i]
      # set of give pair whose distance are shortest


      # print(setFive)
      for j in range(i+1,len(data)):
        # clus2 = []
        clus2 = data[j]
        setFive = np.zeros((5,5))
        setFive[:,4] = 1.0
        # point1 = np.array([0.0, 0.0])
        # point2 = np.array([0.0, 0.0])
        max_distance = 1.0
        max_pair = 4
        for m in range(len(clus1)):
          for n in range(len(clus2)):
            distance = np.sqrt(np.power((clus1[m,0]-clus2[n,0]),2) + np.power((clus1[m,1]-clus2[n,1]),2))
            # if distance < 0.1:
              # print(distance)
            # compare the distance with the maximum in the distance matrix.
            # print("the length of a cluster 2: ", len(clus2))
            # print("the length of a cluster 1: ", len(clus1))
            if distance < max_distance:
              max_distance = distance
              setFive[max_pair,0] = clus1[m,0]
              setFive[max_pair,1] = clus1[m,1]
              setFive[max_pair,2] = clus2[n,0]
              setFive[max_pair,3] = clus2[n,1]
              setFive[max_pair,4] = distance
              for l in range(5):
                if setFive[l,4] > max_distance:
                  max_distance = setFive[l,4]
                  max_pair = l


        # condition to be neighbor is the distances of 5 point pairs are less than threshold 
        if max_distance < 0.01:
          neig_matrix[i,j] = 1.0
          neig_matrix[j,i] = 1.0
        # print(neig_matrix)

    return neig_matrix

  def graph_construct(self, clusterSet, neigh_matrix, line_cooefs, bInterSects):
    cp_neighMatrix = neigh_matrix
    cp_bInterSects = bInterSects
    loop_run = True
    vertex_list = []
    edges = np.array([[0.0, 0.0, 0.0]])
    count = 0
    print(cp_neighMatrix)

    for db in range(len(neigh_matrix)):
      # count += 1
      # find the cluster with minimum neighbors
      min_neigh = len(cp_neighMatrix)
      min_index = len(cp_neighMatrix)
      # print(min_neigh)
      neighNumber = 0
      cVer = 0
      # loop to find a cluster with minimum neighbors to start
      for i1 in range(len(cp_neighMatrix)):
        for j1 in range(len(cp_neighMatrix[0])):
          neighNumber += cp_neighMatrix[i1,j1]
        # print("neighbor's number:", neighNumber)

        if (neighNumber > 0) and (neighNumber < min_neigh):
          min_neigh = neighNumber
          min_index = i1
        neighNumber = 0      
      # print("min neighbor's number is selected:", min_neigh)
      # if neighNumber ==0:
        # loop_run = False
      firstTime_cluster = True

      
      # select the cluster with min neighbor to process
      if min_neigh > 0:  
        for i in range(len(cp_neighMatrix[0])): 
          if not (cp_neighMatrix[min_index,i] == 0.0):
            # print("neighbor's number:", min_index)
            interSect1 = []
            # find intersect between two lines if they are nearly parallel
            if np.absolute(line_cooefs[min_index,0] - line_cooefs[i,0]) < 0.5:
              interSect1 = self.find_InterSecTwoclusters(bInterSects[min_index], bInterSects[i]);
            else: 
              interSect1 = self.twolines_intersect(line_cooefs[min_index], line_cooefs[i])
            # print("neighbor's number:", interSect1)
            se_bInter = []
            # check whether that points lie in the vertex list??
            insideList = False
            
            # if this is the first vertex
            if len(vertex_list) == 0:
              cVer = 0
              dis1, dis2 = self.twodis_twoPoints(interSect1, cp_bInterSects[min_index]);
              if dis1 > dis2:
                # cp_bInterSects[min_index,2] = vertex_list[cVer,0]
                # cp_bInterSects[min_index,3] = vertex_list[cVer,1]
                cp_bInterSects[min_index,2] = interSect1[0]
                cp_bInterSects[min_index,3] = interSect1[1]
                se_bInter = np.array([cp_bInterSects[min_index,0], cp_bInterSects[min_index,1]])
              else:
                cp_bInterSects[min_index,0] = interSect1[0]
                cp_bInterSects[min_index,1] = interSect1[1]
                se_bInter = np.array([cp_bInterSects[min_index,2], cp_bInterSects[min_index,3]])
              # add the first and second vertex
              
              vertex_list = np.array([se_bInter])
              vertex_list = np.append(vertex_list, [interSect1], axis=0)
              # vertex_list = np.array([])
              dis = self.dis_twoPoints(vertex_list[cVer], vertex_list[len(vertex_list)-1])
              # add the first edges
              edges[0,0] = float(cVer)
              edges[0,1] = 1.0
              edges[0,2] = dis

            else:
              for j in range(len(vertex_list)):
                # print("check can the program go in here")
                # if there are vertex close to the intersection
                if self.dis_twoPoints(interSect1, vertex_list[j]) < 0.2:
                  insideList = True
                  cVer = j

                  dis1, dis2 = self.twodis_twoPoints(interSect1, cp_bInterSects[min_index]);
                  if dis1 > dis2:
                    cp_bInterSects[min_index,2] = vertex_list[cVer,0]
                    cp_bInterSects[min_index,3] = vertex_list[cVer,1]
                    se_bInter = np.array([cp_bInterSects[min_index,0], cp_bInterSects[min_index,1]])
                  else:
                    cp_bInterSects[min_index,0] = vertex_list[cVer,0]
                    cp_bInterSects[min_index,1] = vertex_list[cVer,1]
                    se_bInter = np.array([cp_bInterSects[min_index,2], cp_bInterSects[min_index,3]])

                  if firstTime_cluster:
                    # check for the second vertex is in the vertices list
                    inside = False #check inside the list
                    for k in range(len(vertex_list)):
                      if self.dis_twoPoints(se_bInter, vertex_list[k]) < 0.18:
                        print ("this vertex is in the vertices list ")
                        inside = True
                        dis = self.dis_twoPoints(vertex_list[cVer], vertex_list[k])
                        edges = np.append(edges, [[float(cVer), float(k), dis]], axis=0)
                    
                    if not inside:
                      vertex_list = np.append(vertex_list, [se_bInter], axis=0)
                      dis = self.dis_twoPoints(vertex_list[cVer], vertex_list[len(vertex_list)-1])
                      # add the edges
                      edges = np.append(edges, [[float(cVer), float(len(vertex_list)-1), dis]], axis=0)
                    firstTime_cluster = False



              if not insideList:
                vertex_list = np.append(vertex_list, [interSect1], axis=0)
                cVer = len(vertex_list) - 1

                dis1, dis2 = self.twodis_twoPoints(interSect1, cp_bInterSects[min_index]);
                if dis1 > dis2:
                  cp_bInterSects[min_index,2] = vertex_list[cVer,0]
                  cp_bInterSects[min_index,3] = vertex_list[cVer,1]
                  se_bInter = np.array([cp_bInterSects[min_index,0], cp_bInterSects[min_index,1]])
                else:
                  cp_bInterSects[min_index,0] = vertex_list[cVer,0]
                  cp_bInterSects[min_index,1] = vertex_list[cVer,1]
                  se_bInter = np.array([cp_bInterSects[min_index,2], cp_bInterSects[min_index,3]])

                if firstTime_cluster:
                  # check for the second vertex is in the vertices list
                  inside = False #check inside the list
                  for k in range(len(vertex_list)):
                    if self.dis_twoPoints(se_bInter, vertex_list[k]) < 0.18:
                      print ("this vertex is in the vertices list ")
                      inside = True
                      dis = self.dis_twoPoints(vertex_list[cVer], vertex_list[k])
                      edges = np.append(edges, [[float(cVer), float(k), dis]], axis=0)
                  
                  if not inside:
                    vertex_list = np.append(vertex_list, [se_bInter], axis=0)
                    dis = self.dis_twoPoints(vertex_list[cVer], vertex_list[len(vertex_list)-1])
                    # add the edges
                    edges = np.append(edges, [[float(cVer), float(len(vertex_list)-1), dis]], axis=0)
                  firstTime_cluster = False

          # remove the neighbor index
          cp_neighMatrix[min_index,i] = 0.0
    # print(vertex_list)
    print(edges)
    graph = np.zeros((len(vertex_list),len(vertex_list)))
    for k in range(len(edges)):
      row = int(edges[k,0])
      col = int(edges[k,1])
      graph[row, col] = edges[k,2]
      graph[col, row] = edges[k,2]
    # print(cp_neighMatrix)
    # print(graph)
    return vertex_list, edges


    
  def find_InterSecTwoclusters(self, bInSect1, bInSect2): 
    interSect1 = []
    inter_x = 0
    inter_y = 0
    # find a pair of clostest points
    d_min = 1.0
    dis1 = np.sqrt(np.power((bInSect1[0]-bInSect2[0]),2) + np.power((bInSect1[1]-bInSect2[1]),2))
    dis2 = np.sqrt(np.power((bInSect1[0]-bInSect2[2]),2) + np.power((bInSect1[1]-bInSect2[3]),2))
    dis3 = np.sqrt(np.power((bInSect1[2]-bInSect2[0]),2) + np.power((bInSect1[3]-bInSect2[1]),2))
    dis4 = np.sqrt(np.power((bInSect1[2]-bInSect2[2]),2) + np.power((bInSect1[3]-bInSect2[3]),2))
    if dis1 < d_min:
      d_min = dis1
      inter_x = (bInSect1[0]+bInSect2[0])/2
      inter_y = (bInSect1[1]+bInSect2[1])/2
    if dis2 < d_min:
      d_min = dis2
      inter_x = (bInSect1[0]+bInSect2[2])/2
      inter_y = (bInSect1[1]+bInSect2[3])/2
    if dis3 < d_min:
      d_min = dis3
      inter_x = (bInSect1[2]+bInSect2[0])/2
      inter_y = (bInSect1[3]+bInSect2[1])/2
    if dis4 < d_min:
      d_min = dis4
      inter_x = (bInSect1[2]+bInSect2[2])/2
      inter_y = (bInSect1[3]+bInSect2[3])/2
    interSect1 = np.array([inter_x, inter_y])
    return interSect1

  def twodis_twoPoints(self,p1, p2):
    dis1 = np.sqrt(np.power((p1[0]-p2[0]),2)+np.power((p1[1] - p2[1]),2))
    dis2 = np.sqrt(np.power((p1[0]-p2[2]),2)+np.power((p1[1] - p2[3]),2))
    return dis1, dis2

  def dis_twoPoints(self,p1, p2):
    dis = np.sqrt(np.power((p1[0]-p2[0]),2) + np.power((p1[1]-p2[1]),2))
    return dis
   

  # build a open CPP graph
  # def make_vocpp(self, graph, v_s, v_t):
  #     # get the odd nodes from the graph
  #     odd_nodes = graph.odd_nodes
  #     # two boolean variables check whether v_, or v_t belong to the odd vertices list
  #     new_graph = []
  #     vs_in = False
  #     vt_in = False
  #     # new_graph = []
  #     for i in range(len(odd_nodes)):
  #       if v_s == odd_nodes[i]:
  #         vs_in = True 
  #       if v_t == odd_nodes[i]:
  #         vt_in = True
  #     # convert the start and target vertices into odd one if needed
  #     if (not vs_in) and vt_in:
  #       odd_nodes.remove(v_t)
  #       # print(odd_nodes)
  #       pairSet = []
  #       min_cost = 0
  #       min_path = []
  #       select_vertex = 0
  #       for j in range(len(odd_nodes)):
  #         pair = (v_s, odd_nodes[j])
  #         pairSet.append(pair)
  #       for k in range(len(odd_nodes)):
  #         cost, path = dijkstra.find_cost(pairSet[k], graph)
  #         if k ==0:
  #           min_cost = cost
  #           min_path = path
  #           select_vertex = 0
  #         else:
  #           if min_cost > cost:
  #             min_path = path
  #             min_cost = cost
  #             select_vertex = k
  #       # print('selected vertext is: ', select_vertex)
  #       odd_nodes.pop(select_vertex)
  #       # print(odd_nodes)
  #       min_path = np.array([min_path])
  #       new_graph =  add_new_edges(graph, min_path)

  #     if vs_in and (not vt_in):
  #       odd_nodes.remove(v_s)
  #       # print('odd nodes list after reomving the v_s',odd_nodes)
  #       pairSet = []
  #       min_cost = 0
  #       min_path = []
  #       select_vertex = 0
  #       for j in range(len(odd_nodes)):
  #         pair = (v_t, odd_nodes[j])
  #         pairSet.append(pair)
  #       for k in range(len(odd_nodes)):
  #         cost, path = dijkstra.find_cost(pairSet[k], graph)
  #         # print('the cost for each pairs',cost)
  #         if k == 0:
  #           min_cost = cost
  #           min_path = path
  #           select_vertex = 0
  #         else:
  #           if min_cost > cost:
  #             min_path = path
  #             min_cost = cost
  #             select_vertex = k
  #       # print('selected vertext is number: ', select_vertex, 'case 3')
  #       odd_nodes.pop(select_vertex)
  #       # print(odd_nodes)
  #       min_path = np.array([min_path])
  #       new_graph =  add_new_edges(graph, min_path)

  #     if (not vs_in) and (not vt_in):
  #       pairSet = []
  #       min_cost = 0
  #       min_path1 = []
  #       select_vertex = 0
  #       # find a path for the start vertex
  #       for j in range(len(odd_nodes)):
  #         pair = (v_s, odd_nodes[j])
  #         pairSet.append(pair)
  #       # print('check the pair Sets:')
  #       # print(pairSet)
  #       for k in range(len(odd_nodes)):
  #         cost, path = dijkstra.find_cost(pairSet[k], graph)
  #         # print('the cost for each pairs',cost)
  #         if k == 0:
  #           min_cost = cost
  #           min_path1 = path
  #           select_vertex = 0
  #         else:
  #           if min_cost > cost:
  #             min_path1 = path
  #             min_cost = cost
  #             select_vertex = k
  #       # print('selected vertex is number: ', select_vertex, 'case 4')
  #       odd_nodes.pop(select_vertex)
  #       # print(odd_nodes)
  #       min_path = np.array([min_path1])
  #       new_graph1 =  add_new_edges(graph, min_path)

  #       min_path2 = []
  #       pairSet2 = []
  #       # find a path for the start vertex
  #       for j in range(len(odd_nodes)):
  #         pair = (v_t, odd_nodes[j])
  #         pairSet2.append(pair)
  #       # print(pairSet2)
  #       for k in range(len(odd_nodes)):
  #         cost, path = dijkstra.find_cost(pairSet2[k], graph)
  #         # print('the cost for each pairs',cost)
  #         if k == 0:
  #           min_cost = cost
  #           min_path2 = path
  #           select_vertex = 0
  #         else:
  #           if min_cost > cost:
  #             min_path2 = path
  #             min_cost = cost
  #             select_vertex = k
  #       # print('selected vertex is number: ', select_vertex, 'case 4')
  #       odd_nodes.pop(select_vertex)
  #       # print(odd_nodes)
  #       min_path = np.array([min_path2])
  #       new_graph =  add_new_edges(new_graph1, min_path)

  #     # if both start and target are in the odd list already   
  #     oddNodesList = new_graph.odd_nodes
  #     print(oddNodesList)
  #     oddNodesList.remove(v_s)
  #     # print(oddNodesList)
  #     oddNodesList.remove(v_t)
  #     print(oddNodesList)

  #     # find all the combination of the odd nodes
  #     node_pairs_all = list([x for x in itertools.combinations(oddNodesList, 2)])
  #     # print(node_pairs_all)
  #     # find the path set to connect all the pairs
  #     pairs_all_paths = find_node_pair_solutions(node_pairs_all, new_graph)
  #     # print(pairs_all_paths)
  #     # build a unique set of pairs
  #     unique_pair_sets = (x for x in unique_pairs(oddNodesList))
  #     print(unique_pair_sets.next())
      
  #     # find the cheapest route
  #     cheapest_set, min_route = find_minimum_path_set(unique_pair_sets, pairs_all_paths)
  #     # print(min_route)

  #     return add_new_edges(new_graph, min_route)



def main(args):
  
  graphEsti = graphEstimate_server()
  graphEsti.callSpawnServiceTopic()

  
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
