import numpy as np
import math 
from math import ceil

##  WORK IN PROGRESS ##

# This is an adapted version of the file RRTC where we are passing through our occupancy grid data instead of obstacle coordinates

#assumptions:
#og and map are both sqaures
#map has a width of 10 units
#the robot has no dimensions (a point)

class RRTC:
    """
    Class for RRT planning
    """
    class Node:
        """
        RRT Node
        """
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, start=np.zeros(2),
                 goal=np.array([120,90]),
                 width = 10,
                 height=10,
                 og_width = 10,
                 og_height = 10,
                 og_resolution = 1,
                 og_data = [],
                 expand_dis=3.0, 
                 path_resolution=0.5, 
                 max_points=200,
                 rob_width = 1.0,
                 rob_height = 1.0):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        width, height: of the map the occuapncy grid will be fitted to
        og_height, og_width, og_resolution, og_data: height, width, resolution and data of the occupancy grid
        expand_dis: min distance between random node and closest node in rrt to it
        path_resolion: step size to considered when looking for node to expand
        rob_width: the width of our robot
        rob_height: the height of our robot
        
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.width = width
        self.height = height
        self.resolution = og_resolution
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_nodes = max_points
        self.og_data = og_data
        self.rob_width = rob_width
        self.rob_height = rob_height
        self.start_node_list = [] # Tree from start
        self.end_node_list = [] # Tree from end

        
    def planning(self):
        """
        rrt path planning
        """
        self.start_node_list = [self.start]
        self.end_node_list = [self.end]
        while len(self.start_node_list) + len(self.end_node_list) <= self.max_nodes:
        
            # sample and add a node in the start tree
            rnd_node = self.get_random_node()
            expansion_ind = self.get_nearest_node_index(self.start_node_list,rnd_node)
            expansion_node = self.start_node_list[expansion_ind] ## closest node in tree to rnd_node

            nearby_node = self.steer(expansion_node,rnd_node,self.expand_dis) ## get node closer to rnd_node from expansion_node

            # add nearby_node to start_list if it is collision free 
            if self.is_collision_free(nearby_node):
                self.start_node_list.append(nearby_node)
            
            # check whether trees can be connected
            expansion_ind = self.get_nearest_node_index(self.end_node_list, nearby_node) 
            expansion_node = self.end_node_list[expansion_ind]
                
            d, thet = self.calc_distance_and_angle(expansion_node, nearby_node)

            # add the node that connects the trees and generate the path
            if d < self.expand_dis:
                close_node = self.steer(expansion_node,nearby_node,self.expand_dis)

                if self.is_collision_free(close_node):
                    self.end_node_list.append(close_node)
                    
                    return self.generate_final_course(len(self.start_node_list) - 1, len(self.end_node_list) - 1)
            
            
            # sample and add a node in the end tree
            rnd_node = self.get_random_node()
            expansion_ind = self.get_nearest_node_index(self.end_node_list,rnd_node)
            expansion_node = self.end_node_list[expansion_ind] ## closest node in tree to rnd_node

            nearby_node = self.steer(expansion_node,rnd_node,self.expand_dis) ## get node closer to rnd_node from expansion_node

            # add nearby_node to end_list if it is collision free
            if self.is_collision_free(nearby_node):
                self.end_node_list.append(nearby_node)

            # swap start and end trees
            temp = self.start_node_list
            self.start_node_list = self.end_node_list
            self.end_node_list = temp
        
    
        return None  # cannot find path
    
    def steer(self, from_node, to_node, extend_length=float("inf")):
        """
        Given two nodes from_node, to_node, this method returns a node new_node such that new_node 
        is “closer” to to_node than from_node is.
        """
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        cos_theta, sin_theta = np.cos(theta), np.sin(theta)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        # How many intermediate positions are considered between from_node and to_node
        n_expand = math.floor(extend_length / self.path_resolution)

        # Compute all intermediate positions
        for _ in range(n_expand):
            new_node.x += self.path_resolution * cos_theta
            new_node.y += self.path_resolution * sin_theta
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = from_node

        return new_node

    def is_collision_free(self, new_node):
        """
        Determine if nearby_node (new_node) is in the collision-free space.
        """
        if new_node is None:
            return True
        
        points = np.vstack((new_node.path_x, new_node.path_y)).T
        for obs in self.obstacle_list:
            in_collision = obs.is_in_collision_with_points(points)
            if in_collision:
                return False
        
        return True  # safe

    def new_node_is_free(self,new_node,og_width,og_height,og_resolution,og_data,map_width, map_height):

        og_index = self.map_to_og_conversion(new_node=new_node,og_width=og_width,og_height=og_height,og_resolution=og_resolution,map_height=map_height,map_width=map_width)
        
        if og_data(og_index) ==0:
            return True
        else:
            return False 

       
    def map_to_og_conversion(self,point,og_width,og_height,og_resolution,map_width, map_height):
        position = np.array([point.x,point.y])
        ratio = og_width/10 #for now just assuming width of the map is 10, also assuming both og and map is square
        
        og_column = ceil(position(0)*ratio)
        og_row = ceil(position(1)*ratio)
        og_index = og_column + og_width*(og_row-1)

        
        return og_index

    def map_to_og_matrix_conversion(self,point,og_width,og_height,map_width, map_height):
        position = np.array([point.x,point.y])
        ratio = og_width/map_width#for now just assuming width of the map is 10, also assuming both og and map is square
        
        og_column = ceil(position(0)*ratio)
        og_row = ceil(position(1)*ratio)

        return [og_column,og_row]

    def og_matrix_to_map_conversion(self,og_grid,og_width,og_height,map_width,map_height):
        ratio = og_width/map_width
        x = og_grid[0]/ratio
        y = og_grid[1]/ratio
        return self.Node(x,y)

    def point_is_on_right_parameter_of_grid_square(self,point,og_width,og_height, map_width, map_height):
        position = np.array([point.x,point.y])
        ratio = og_width/map_width

        if ceil(position(0)*ratio) == position(0)*ratio:
            return True
        else:
            return False

    def point_is_on_bottom_parameter_of_grid_square(self,point,og_width,og_height,map_width,map_height):
        position = np.array([point.x,point.y])
        ratio = og_width/map_width

        if ceil(position(1)*ratio) == position(1)*ratio:
            return True
        else:
            return False

    def minor_path_is_collision_free(self,new_node,prev_node,og_height,og_width,map_width,map_height):


            #gradient has to be positive probs
        start = prev_node
        end = new_node
        ratio = og_width/map_width
        starting_grid = self.map_to_og_matrix_conversion(self = self,point=start,og_width=og_width,og_height=og_height,map_height= map_height,map_width= map_width)
        end_grid = self.map_to_og_matrix_conversion(self=self,point=end,og_width=og_width,og_height=og_height,map_height= map_height,map_width= map_width )
        on_right = self.point_is_on_right_parameter_of_grid_square (point= start,og_width= og_width,og_height= og_height, map_width= map_width,map_height= map_height)
        on_bottom = self.point_is_on_bottom_parameter_of_grid_square(point= start,og_width= og_width,og_height= og_height, map_width= map_width,map_height= map_height)
        gradient = end.y - start.y / (end.x - start.x)
        delta_x = end.x - start.x
        delta_y = end.y-start.y
        step_size = map_width/og_width

        #To decide the starting square of the occupancy grid if the point is between two grid squares
        if on_right or on_bottom:
            if end.x > start.x & end.y < start.y:
                if on_right:
                    starting_grid[0] = starting_grid[0] + 1
                if on_bottom:
                    starting_grid[1] = starting_grid[1] - 1 
            elif end.x > start.x:
                if on_right:
                    starting_grid[0] = starting_grid[0] + 1

            elif end.y < start.y:
                if on_bottom:
                    starting_grid[1] = starting_grid[1] - 1 
        
            
        last_added_grid = starting_grid
        grids_to_check = np.array([last_added_grid])
        
        if abs(gradient)<1:
            if delta_x >0 and delta_y >0:
                if on_right and on_bottom:
                    d = gradient
                    
                    while last_added_grid != end_grid:
                        last_added_grid[0] = last_added_grid[0]+delta_x/abs(delta_x)
                        np.append(grids_to_check,[last_added_grid],0)
                        d = d + gradient
                        if d >=1:
                            d = d-1
                            last_added_grid[1] = last_added_grid[1]+ delta_y/abs(delta_y)
                
                elif on_right:
                    origin_node = self.og_matrix_to_map_conversion(starting_grid,og_width=og_width,og_height=og_height,map_width=map_width,map_height=map_height)
                    d = gradient + ((start.y - origin_node.y) * ratio)
                    while last_added_grid != end_grid:
                        if d >=1:
                            d = d-1
                            last_added_grid[1] = last_added_grid[1]+delta_y/abs(delta_y)
                        last_added_grid[0] = last_added_grid[0]+delta_x/abs(delta_x)
                        np.append(grids_to_check,[last_added_grid],0)
                        d = d + gradient
                
                elif on_bottom:
                    origin_node = self.og_matrix_to_map_conversion(starting_grid,og_width=og_width,og_height=og_height,map_width=map_width,map_height=map_height)
                    d = gradient * (origin_node.x - start.x) *ratio #has to be less than 1
                    while last_added_grid != end_grid:
                        if d >=1:
                            d = d-1
                            last_added_grid[1] = last_added_grid[1]+delta_y/abs(delta_y)
                        last_added_grid[0] = last_added_grid[0]+delta_x/abs(delta_x)
                        np.append(grids_to_check,[last_added_grid],0)
                        d = d + gradient
                else:
                    origin_node = self.og_matrix_to_map_conversion(starting_grid,og_width=og_width,og_height=og_height,map_width=map_width,map_height=map_height)
                    d = ((start.y - origin_node.y) * ratio) + gradient * (origin_node.x - start.x) *ratio
                    # d = abs(0.5 - delta_y/2*abs(delta_y) - (start.y-origin_node.y)*ratio) + gradient * abs(0.5 - delta_x/2*abs(delta_x) - (origin_node.x - start.x) * ratio)
                    # d = (abs(0.5 +0.5 - (0.8) ) + abs(0.5 +0.5 -(0.4)) *  0.375 = 0.2+0.225 = 0.425
                    # 
                    # d = abs(0.5 -0.5 - 0.8) + abs (0.5+0.5 - 0.4)**0.375 = 0.8+0.225= 1.025
                    while last_added_grid != end_grid:
                        if d >=1:
                            d = d-1
                            last_added_grid[1] = last_added_grid[1]+delta_y/abs(delta_y)
                        last_added_grid[0] = last_added_grid[0]+delta_x/abs(delta_x)
                        np.append(grids_to_check,[last_added_grid],0)
                        d = d + gradient

            elif delta_x >0 and delta_y <0:
                delta_x = 5

                    
                    
                    



        #starting_grid[0] = starting_grid[0] + 1
        #if end.y < start.y:
        #starting_grid[1] = starting_grid[1] - 1
        # 
        #         

    #First step: decide whether it is mostly vertical, mostly horizontal, or a 45er
    #Case mostly vertical or mostly horizontal:
        #get the gradient
        #Case at a cross point between grids:
            #make an array of grids coordinates, with first element being the starting grid 
            # starting gird is determined by the direction of the movement [0-90) grid to the right of the point grid, [90-180) same grid as the point grid, 
            # [180-270) grid bottom of the point grid, [270-360) grid bottom of and right of the point grid

        
        
        return 0
    def generate_final_course(self, start_mid_point, end_mid_point):
        """
        Reconstruct path from start to end node
        """
        # First half
        node = self.start_node_list[start_mid_point]
        path = []
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        
        # Other half
        node = self.end_node_list[end_mid_point]
        path = path[::-1]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        x = self.width * np.random.random_sample()
        y = self.height * np.random.random_sample()
        rnd = self.Node(x, y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):        
        # Compute Euclidean disteance between rnd_node and all nodes in tree
        # Return index of closest element
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta     