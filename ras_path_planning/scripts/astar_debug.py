#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Vector3
import numpy as np
import matplotlib.pyplot as plt
from math import fabs
from copy import deepcopy

adjacent_squares = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]
adjacent_diagonal_squares = [(-1, -1), (-1, 1), (1, -1), (1, 1)]

class GridMap():
    def __init__(self, data=[], width=0, height=0, resolution=0, origin_x=0, origin_y=0):
        self.data = data
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

class Map():
    def __init__(self):
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.callback_gridmap)
        self.info = GridMap()
        rospy.sleep(3)
        self.getMap()

    def callback_gridmap(self, msg):
        self.info.data = msg.data
        self.info.width = msg.info.width
        self.info.height = msg.info.height
        self.info.resolution = msg.info.resolution
        self.info.origin_x = msg.info.origin.position.x
        self.info.origin_y = msg.info.origin.position.y

    def getMap(self):
        return self.info

class Node():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, next):
        return self.position == next.position

def Heuristic(x_current, y_current, x_target, y_target):
    return (((x_target - x_current) ** 2) + ((y_target - y_current) ** 2))

def similarity_check(child, close_child):
    x_cc = close_child.position[0]
    y_cc = close_child.position[1]
    x = child.position[0]
    y = child.position[1]

    return ((x_cc - x)**2 + (y_cc - y)**2)**0.5

def astar(map, start, end, count):

    # Adaptive threshold for exploration
    (x_start,y_start) = start
    (x_end,y_end) = end
    if Heuristic(x_start,y_start,x_end,y_end)**0.5 < 20:
        threshold = int(6*Heuristic(x_start,y_start,x_end,y_end)**0.5)
    else:
        threshold = int(3*Heuristic(x_start,y_start,x_end,y_end)**0.5)

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize lists
    living_list = [] # states that have been encountered, but possibly have unvisited next states
    dead_list = []   # states that cannot contribute to the search

    currentNodes = []

    # Add start node to the living list
    living_list.append(start_node)

    # Loop until we find goal
    while len(living_list) > 0:

        # Get the current node
        current_node = living_list[0]
        current_idx = 0
        for idx, item in enumerate(living_list):
            currentNodes.append(item)
            if item.f < current_node.f:
                current_node = item
                current_idx = idx

        # Pop current node off the living list and add it to the dead list
        living_list.pop(current_idx)
        dead_list.append(current_node)

        # If we find goal, return reversed path
        if current_node == end_node or count > 3000:
            path = []
            controls_x = []
            controls_y = []
            current = current_node
            while current is not None:
                path.append(current.position)
                controls_x.append(current.position[0])
                controls_y.append(current.position[1])
                current = current.parent
            #return path[::-1] # Return reversed path

            node_coords_x = []
            node_coords_y = []
            for node in currentNodes:
                node_coords_x.append(node.position[0])
                node_coords_y.append(node.position[1])
            return path[::-1], controls_x[::-1], controls_y[::-1], node_coords_x, node_coords_y

        # Generate children
        children = []
        for new_pos in adjacent_squares:

            # Get position of new node
            new_node = (current_node.position[0] + new_pos[0], current_node.position[1] + new_pos[1])

            # Check if within range
            if new_node[1] > (len(map) - 1) or new_node[1] < 0 or new_node[0] > (len(map[len(map)-1]) -1) or new_node[0] < 0:
                continue

            # Check if no obstacle
            if map[new_node[1]][new_node[0]] > 25:
                continue

            # Create new node and set current node as parent
            new_node = Node(current_node, new_node)

            # Add the new node (child) to children
            children.append(new_node)

        # Loop through children
        for child in children:

            # Check if child is equal to child in the dead list
            discard_child = False
            for dead_child in dead_list:
                dist = similarity_check(child, dead_child)
                if child == dead_child or dist > threshold:
                # if child == dead_child:
                    discard_child = True
                    break

            if discard_child == True:
                continue


            # Calculate costs
            if map[child.position[1]][child.position[0]] != 0:
                child.g = current_node.g + 200
            else:
                child.g = current_node.g + 1
            # Attempt to penalise unnecessary diagonal steps. Didn't work out well, smoothing is probably smarter
            # if (child.position[1]-current_node.position[1], child.position[0]-current_node.position[0]) in adjacent_diagonal_squares:
            #     child.g += 200
            # child.g = current_node.g + 1
            child.h = Heuristic(child.position[0], child.position[1], end_node.position[0], end_node.position[1])
            child.f = child.g + child.h

            # Check if child is already in the living list and check g-cost
            discard_child = False
            for living_node in living_list:
                dist = similarity_check(child, living_node)
                if child == living_node or dist > threshold and child.g > living_node.g:
                    discard_child = True
                    break

            if discard_child == True:
                continue

            # Add the child to the living list
            living_list.append(child)
        print(count)
        count+=1

def raytrace(start, end):
# Get cells between two cells
    (start_x, start_y) = start
    (end_x, end_y) = end
    x = start_x
    y = start_y
    (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
    n = dx + dy
    x_inc = 1
    if end_x <= start_x:
        x_inc = -1
    y_inc = 1
    if end_y <= start_y:
        y_inc = -1
    error = dx - dy
    dx *= 2
    dy *= 2

    traversed = []
    for i in range(0, int(n)):
        traversed.append((int(x), int(y)))

        if error > 0:
            x += x_inc
            error -= dy
        else:
            if error == 0:
                traversed.append((int(x + x_inc), int(y)))
            y += y_inc
            error += dx

    traversed.append((end_x,end_y))

    return traversed

def pathsmoothing(path, gridmap):
    N = len(path)-1
    new_path = []
    new_path.append(path[0])

    currentCell = 0
    nextCell = 2
    free = True
    while free and nextCell < N:
            cells = raytrace(path[currentCell],path[nextCell])
            for (x,y) in cells:
                if gridmap[y][x] > 25:
                    free = False
                    currentCell = nextCell-1
                    (x_new,y_new) = path[currentCell]
                    new_path.append((x_new,y_new))
                    break
            if free:
                nextCell += 1
            else:
                nextCell += 2
                free = True

    new_path.append(path[-1])

    return new_path[::-1]

# def pathsmoothing(path, gridmap):
#     N = len(path)-1
#     new_path = []
#     new_path.append(path[0])
#     print("N: %d" , N)
#     # print(path)
#     currentCell = 0
#     notFirstColl = False
#     while currentCell < N-1:
#         for next in range(currentCell+2,N):
#             # print("next: %d" , next)
#             print("current in the loop: ", path[currentCell])
#             cells = raytrace(path[currentCell], path[next])
#             collision = False
#             for (x,y) in cells:
#                 if gridmap[y][x] != 0:
#                     # print("Collision on: " , next)
#                     collision = True
#                     break
#             if collision and notFirstColl:
#                 continue
#             elif not collision and notFirstColl:
#                 print("innan first coll", path[next])
#                 currentCell_temp = next
#                 # print("Temp: %d" , currentCell_temp)
#                 (x_new,y_new) = path[currentCell_temp]
#             elif collision and not notFirstColl:
#                 notFirstColl = True
#                 print("first coll: ", path[next], path[next-1])
#                 currentCell_temp = next-1
#                 # print("Temp: %d" , currentCell_temp)
#                 (x_new,y_new) = path[currentCell_temp]
#             else:
#                 currentCell = currentCell+1
#                 (x_new,y_new) = path[currentCell]
#
#         print("current: " , currentCell, path[currentCell])
#         # rospy.sleep(3)
#         if currentCell == currentCell_temp:
#             # currentCell = path.index(path[currentCell])+1
#             (x_new,y_new) = path[currentCell]
#             print("hej")
#             print("nextCell after  current: ",path[currentCell+1])
#             print("nneextCell after  current: ",path[currentCell+2])
#             ray = raytrace(path[currentCell], path[currentCell+2])
#             print(ray)
#             for (x,y) in ray:
#                 print(gridmap[y][x])
#             # sys.exit(1)
#             # return new_path[::-1]
#         else:
#             currentCell = currentCell_temp
#
#         new_path.append((x_new,y_new))
#
#     return new_path[::-1]

def smooth(path, weight_data=0.2, weight_smooth=0.8, tolerance=0.000001):
    """
    Creates a smooth path for a n-dimensional series of coordinates.
    Arguments:
        path: List containing coordinates of a path
        weight_data: Float, how much weight to update the data (alpha)
        weight_smooth: Float, how much weight to smooth the coordinates (beta).
        tolerance: Float, how much change per iteration is necessary to keep iterating.
    Output:
        new: List containing smoothed coordinates.

    w_data = 0.1 and weight_smooth = 0.9 works well without any other smoothing
    """
    pathlist = []
    for (x,y) in path:
        pathlist.append([x,y])
    path = pathlist


    new = deepcopy(path)
    dims = len(path[0])
    change = tolerance

    while change >= tolerance:
        change = 0.0
        for i in range(1, len(new) - 1):
            for j in range(dims):

                x_i = path[i][j]
                y_i, y_prev, y_next = new[i][j], new[i - 1][j], new[i + 1][j]

                y_i_saved = y_i
                y_i += weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2 * y_i))
                new[i][j] = (y_i)

                change += abs(y_i - y_i_saved)
    new_path = []
    for row in new:
        new_path.append((row[0],row[1]))


    return new

def validgoal(map,goal):
    (x,y) = goal
    if map[y][x] > 25:
        print("Goal is not reachable, try again...")
        return False
    else:
        return True

class mainClass():
    def __init__(self):
        goal_sub = rospy.Subscriber("move_base_simple/goal",PoseStamped, self.goal_callback)
        start_sub = rospy.Subscriber("localisation/position", Vector3, self.start_callback)
        self.goal_flag = False
        self.start_flag = False
        self.main()

    def goal_callback(self, data):
        self.goal = data
        self.goal_flag = True

    def start_callback(self, data):
        self.start_pos = data
        self.start_flag = True

    def calcDist(self, x_current, y_current, x_target, y_target):
        return (((x_target - x_current) ** 2) + ((y_target - y_current) ** 2))**0.5

    def main(self):
        path_pub = rospy.Publisher("/aPath", Path, queue_size=10)
        map = Map()
        gridmap = np.array(map.info.data).reshape(map.info.height,map.info.width)
        gridmap = gridmap.tolist()

        while not rospy.is_shutdown():
            # rospy.spin()
            # x1 = 0.2
            # y1 = 0.2

            # print("waiting for goal...")
            if self.goal_flag:
                print("Goal received!")
                self.goal_flag = False
            else:
                continue

            if self.start_flag:
                print("Starting position received!")
                self.start_flag = False
            else:
                continue


            # print("waiting for starting position...")
            # start_pos = rospy.wait_for_message("odometry/odometry", Vector3)
            # print("Starting position received!")

            x1 = self.start_pos.x
            y1 = self.start_pos.y

            x2 = self.goal.pose.position.x
            y2 = self.goal.pose.position.y

            # print(x1,y1,x2,y2)
            # print(map.info.resolution)

            map_idx_x1 = int((x1-map.info.origin_x)/map.info.resolution);
            map_idx_y1 = int((y1-map.info.origin_y)/map.info.resolution);

            map_idx_x2 = int((x2-map.info.origin_x)/map.info.resolution);
            map_idx_y2 = int((y2-map.info.origin_y)/map.info.resolution);

            start = (map_idx_x1, map_idx_y1)
            end   = (map_idx_x2, map_idx_y2)

            valid_goal = validgoal(gridmap,end)

            if not valid_goal:
                continue

            count = 0
            try:
                path, traj_x, traj_y, nodes_x, nodes_y = astar(gridmap, start, end, count)
            except:
                print("A* failed, trying again.")


            print("A* done")
            times = [t for t in range(len(path))]

            plt.plot(traj_x, traj_y)
            plt.scatter(nodes_x, nodes_y)

            # path = pathsmoothing(path,gridmap)
            path = pathsmoothing(path[::-1],gridmap)
            #path = pathsmoothing(path[::-1],gridmap)
            # path = smooth(path)


            path_msg = Path()
            path_msg.header.frame_id = "base_map"
            for (x,y) in path:
                pose = PoseStamped()
                pose.pose.position.x = x*map.info.resolution + map.info.origin_x + map.info.resolution*0.5
                pose.pose.position.y = y*map.info.resolution + map.info.origin_y + map.info.resolution*0.5
                path_msg.poses.append(pose)

            path_pub.publish(path_msg)
            try:
                plt.show()
            except:
                print("ERRORORORORO")

if __name__ == '__main__':
    rospy.init_node('ras_path_planning')
    try:
        mainClass()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
