#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np

np.random.seed(444)


class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()
        self.step = 0.15
        self.radius_multiplier = 5
        self.radius = self.step * self.radius_multiplier
        self.heuristics = {self.start: 0}  #define heuristics dictionary, heuristics is value of distance from start point
        # points.py: st = Point(10.0, 10.0), en = Point(15.0, 15.0)

    def check_if_valid(self, a, b):
        """
        Checks if the segment connecting a and b lies in the free space.
        :param a: point in 2D
        :param b: point in 2D
        :return: boolean
        """
        in_free_space = True
        # linear function y= alpha * x + beta
        alpha = (b[1]-a[1])/(b[0]-a[0])
        beta = a[1] - a[0] * alpha
        if a[0] == b[0]:
            step = abs(a[1]-b[1])/100
            test_x = a[0]
        else:
            step = abs(a[0]-b[0])/100

        for i in range(0, 101):
            if a[0] < b[0]:
                test_x = a[0] + i * step
                test_y = test_x * alpha + beta
            elif a[0] > b[0]:
                test_x = a[0] - i * step
                test_y = test_x * alpha + beta
            else:
                if a[1] < b[1]:
                    test_y = a[1] + i * step
                else:
                    test_y = a[1] - i * step

            if self.map[int(test_y/self.resolution), int(test_x/self.resolution)] > 50:
                in_free_space = False
                break
        return in_free_space

    def random_point(self):

        """
        Draws random point in 2D
        :return: point in 2D
        """

        x = np.random.random() * self.width
        y = np.random.random() * self.height
        return x, y

    def find_closest(self, pos):
        """
        Finds the closest vertex in the graph to the pos argument

        :param pos: point id 2D
        :return: vertex from graph in 2D closest to the pos
        """
        dist_squared = 1000 # square root is not necessary to compare distances, therefore it has been omitted to reduce calculations
        for parent in self.parent:
            if dist_squared > (pos[0] - parent[0]) ** 2 + (pos[1] - parent[1]) ** 2:
                dist_squared = (pos[0] - parent[0]) ** 2 + (pos[1] - parent[1]) ** 2
                closest = parent
        return closest

    def new_pt(self, pt, closest):
        """
        Finds the point on the segment connecting closest with pt, which lies self.step from the closest (vertex in graph)

        :param pt: point in 2D
        :param closest: vertex in the tree (point in 2D)
        :return: point in 2D
        """

        p_x, p_y = pt[0], pt[1]
        q_x, q_y = closest[0], closest[1]
        x = p_x - q_x
        y = p_y - q_y
        vector = np.array([x, y])
        z = np.linalg.norm(vector)
        point_x = (x * self.step / z) + q_x
        point_y = (y * self.step / z) + q_y

        return point_x, point_y

    def calc_dist(self, pt1, pt2):
        return ((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2) ** 0.5

    def find_new_parent(self, pt):
        for parent in self.parent:
            if parent is not pt and self.calc_dist(parent, pt) < self.radius:
                new_heuristics = self.heuristics[parent] + self.calc_dist(parent, pt)
                if new_heuristics < self.heuristics[pt] and self.check_if_valid(parent, pt):
                    self.parent[pt] = parent
                    self.heuristics[pt] = new_heuristics

    def search(self):
        """
        RRT search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
        """

        end_reached = False
        self.parent[self.start] = None

        print("Looking for endpoint")
        while not end_reached:
            random_point = self.random_point()
            closest_point = self.find_closest(random_point)
            new_point = self.new_pt(random_point, closest_point)
            if self.check_if_valid(closest_point, new_point):
                self.parent[new_point] = closest_point
                self.heuristics[new_point] = self.heuristics[self.parent[new_point]] + self.calc_dist(new_point, self.parent[new_point])
                self.find_new_parent(new_point)
                if self.check_if_valid(new_point, self.end):
                    self.parent[self.end] = new_point
                    end_reached = True
            self.publish_search()
            rp.sleep(0.5)
        print("End found")

        #restore path
        start_reached = False
        considered_node = self.end
        path = [considered_node]
        print("Restore path")
        while not start_reached:
            considered_node = self.parent[considered_node]
            path.append(considered_node)
            if considered_node == self.start:
                start_reached = True
        print("Path restored")
        self.publish_path(path)
        while not rp.is_shutdown():
            rp.sleep(0.01)


if __name__ == '__main__':
    rrt = RRT()
    rrt.search()
