"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""

import math
import random

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, phi):
            self.x = x
            self.y = y
            self.phi = phi
            self.path_x = []
            self.path_y = []
            self.path_phi = []
            self.parent = None

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[3])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.phi_rand = rand_area[2]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []

    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            # return the index of the nearest node
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            # if self.check_collision(new_node, self.obstacle_list):
            if self.ara_check_collision(new_node, boudnaries):
                self.node_list.append(new_node)

            # if animation and i % 5 == 0:
            #     self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y
                                      self.node_list[-1].phi) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            # if animation and i % 5:
            #     self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta, d_phi = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        new_node.path_phi = [new_node.phi]

        if extend_length > d:
            extend_length = d
        # find the number of step to extend by rounding down ...
        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.phi += self.path_resolution*d_phi
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
            new_node.path_phi.append(new_node.phi)

        d, _, _ = self.calc_distance_and_angle(new_node, to_node)
        
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.path_phi.append(to_node.phi)
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.phi = to_node.phi

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y, self.end.phi]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y, node.phi])
            node = node.parent
        path.append([node.x, node.y, node.phi])

        return path

    def calc_dist_to_goal(self, x, y, phi):
        dx = x - self.end.x
        dy = y - self.end.y
        dphi = phi - self.end.phi
        dis = math.hypot(dx, dy)
        return dis + dphi

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(-self.phi_rand, self.phi_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y, self.end.phi)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 + (node.phi - rnd_node.phi)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_collision(node, obstacleList):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size**2:
                return False  # collision

        return True  # safe

    # @staticmethod
    def ara_check_collision(self, node, data):

        if node is None:
            return False
        # calculate all the other conner of the robot
        roPos = self.robot_positions(node)
        # calculate the center point of the clusters
        cenPointSet = []
        for i in range(len(data)):
            data1 = data[i]
            for j in range(len(data1)):
                cen_x = data[j,0]
                cen_y = data[j,1]
            cen_x = cen_x/len(data1)
            cen_y = cen_y/len(data1)
            if i ==0:
                cenPointSet = np.array([[cen_x, cen_y]])
            else:
                cenPointSet = np.append(cenPointSet, [[cen_x, cen_y]], axis=0)

        # check in the boudnary for each robot point
        for pos in range(len(roPos)):
            # select 3 closest clusters around the robot point by their center
            clus_index = np.array([0,0,0])
            clus_dis = np.array([1.5,1.5,1.5])
            d_max = 1.5
            dmax_index = 0
            if len(cenPointSet) < 3:
                clus_index[2] = -1
                clus_disp[2] = 0.0
            for i in range(len(cenPointSet)):
                dis = self.distance(roPos[pos], cenPointSet[i])
                if dis < d_max:
                    d_max = dis
                    clus_dis[dmax_index] = dis
                    clus_index[dmax_index] = i
                    for k in range(len(clus_dis)):
                        if d_max < clus_dis[k]:
                            d_max = clus_dis[k]
                            dmax_index = k
            # check the robot point whether it belongs to one of the clusters
            for j in range(len(clus_index)):
                if not (clus_index[j] == -1):
                    data2 = data[clus_index[j]]
                    neigh_index = np.array([0,0,0,0])
                    dist4neighbors = np.array([1.0,1.0,1.0,1.0])
                    max_distance = 1.5
                    point_index = 0
                    # find 4 closest point around the robot point in each cluster
                    for k in range(len(data2)):
                        dis = self.distance(roPos[pos], data2[k])
                        if dis < max_distance:
                            max_distance = dis
                            dist4neighbors[point_index] = dis
                            neigh_index[point_index] = k
                            for m in range(len(dist4neighbors)):
                                if max_distance < dist4neighbors[m]:
                                    max_distance = dist4neighbors[m]
                                    point_index = m
                    # compare the 4 distance of 4 points and the robot points to the center points
                    d_nc = np.array([1.0,1.0,1.0,1.0])
                    for g in range(len(neigh_index)):
                        d_nc1 = self.distance(cenPointSet[clus_index[j]], data2[neigh_index[g]])
                        d_nc[g] = d_nc1
                    d_rpc = self.distance(cenPointSet[clus_index[j]], roPos[pos])
                    
                    # boolean check set
                    bcs = [True, True, True, True]
                    for h in range(len(d_nc)):
                        if np.absolute(d_rpc - d_nc[h]) > 0.01:
                            bcs[h] = False
                    check = True
                    for u in range(len(bcs)):
                        check = check and bcs[u]

                    if check:
                        return True # Safe 
        return False  # collision

    def distance(self, pos1, pos2):
        return np.sqrt(np.power((pos1[0]-pos2[0]),2)+np.power((pos1[1]-pos2[1]),2))


    def robot_positions(self, node)
        x = node.x
        y = node.y
        phi = node.phi
        d = 0.4 # distance between two feet
        l = 0.18 # length of the foot
        w = 0.22 # width of the foot
        dc = np.sqrt(np.power((l/2),2) + np.power((w/2),2))
        alpha = np.arctan(w/d)
        p1_x = x + dc*np.cos(-alpha + phi)
        p1_y = y + dc*np.sin(-alpha + phi)
        PoRo = np.array([[p1_x, p1_y]])

        p2_x = x + dc*np.cos(alpha + phi)
        p2_y = y + dc*np.sin(alpha + phi)
        PoRo = np.append(PoRo, [[p2_x, p2_y]], axis=0)

        p3_x = x + dc*np.cos(np.pi - alpha + phi)
        p3_y = y + dc*np.sin(np.pi - alpha + phi)
        PoRo = np.append(PoRo, [[p3_x, p3_y]], axis=0)

        p4_x = x + dc*np.cos(-np.pi + alpha + phi)
        p4_y = y + dc*np.sin(-np.pi + alpha + phi)
        PoRo = np.append(PoRo, [[p4_x, p4_y]], axis=0)

        p5_x = -d*cos(phi) + x + dc*np.cos(-alpha + phi)
        p5_y = -d*sin(phi) + y + dc*np.sin(-alpha + phi)
        PoRo = np.append(PoRo, [[p5_x, p5_y]], axis=0)

        p6_x = -d*cos(phi) + x + dc*np.cos(alpha + phi)
        p6_y = -d*sin(phi) + y + dc*np.sin(alpha + phi)
        PoRo = np.append(PoRo, [[p6_x, p6_y]], axis=0)

        p7_x = -d*cos(phi) + x + dc*np.cos(np.pi - alpha + phi)
        p7_y = -d*sin(phi) + y + dc*np.sin(np.pi - alpha + phi)
        PoRo = np.append(PoRo, [[p7_x, p7_y]], axis=0)

        p8_x = -d*cos(phi) + x + dc*np.cos(-np.pi + alpha + phi)
        p8_y = -d*sin(phi) + y + dc*np.sin(-np.pi + alpha + phi)
        PoRo = np.append(PoRo, [[p8_x, p8_y]], axis=0)

        return PoRo
    
    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        delta_phi = to_node.phi - from_node.phi
        phi_res = delta_phi/d
        theta = math.atan2(dy, dx)
        return d, theta, phi_res


def main(gx=6.0, gy=10.0):
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(
        start=[0, 0],
        goal=[gx, gy],
        rand_area=[-2, 15],
        obstacle_list=obstacleList)
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()
