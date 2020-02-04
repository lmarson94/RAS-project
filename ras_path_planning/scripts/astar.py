#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Vector3, Point
import numpy as np
from math import fabs
import sys
from copy import deepcopy
import heapq

x_max = 0
y_max = 0

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
        self.getMap()

    def callback_gridmap(self, msg):
        self.info.data = msg.data
        self.info.width = msg.info.width
        self.info.height = msg.info.height
        self.info.resolution = msg.info.resolution
        self.info.origin_x = msg.info.origin.position.x
        self.info.origin_y = msg.info.origin.position.y

    def getMap(self):
        while not self.info.data:
            continue
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

    def __lt__(self, other):
        if isinstance(other,Node):
            return self.f < other.f
        else:
            return self.g > other.g

def Heuristic(x_current, y_current, x_target, y_target):
    return ((x_target - x_current) ** 2) + ((y_target - y_current) ** 2)

def similarity_check(child, close_child):
    x_cc = close_child.position[0]
    y_cc = close_child.position[1]
    x = child.position[0]
    y = child.position[1]

    return ((x_cc - x)**2 + (y_cc - y)**2)**0.5

def inBounds(x, y):
    if x >= 0 and x < x_max and y >= 0 and y < y_max:
        return True
    else:
        return False

def EucDist(x_current, y_current, x_target, y_target):
    return (((x_target - x_current) ** 2) + ((y_target - y_current) ** 2))**0.5

def getAdjacentCells(map, current_node, end):
    x = current_node.position[0]
    y = current_node.position[1]
    dist = 2
    adj_cells = []
    while not len(adj_cells):
        for row in range(dist):
            for col in range(dist):
                if inBounds(x+col,y+row) and map[y+row][x+col] <= 25:
                    adj_cells.append((x+col,y+row))
                if inBounds(x+col,y-row) and map[y-row][x+col] <= 25:
                    adj_cells.append((x+col,y-row))
                if inBounds(x-col,y+row) and map[y+row][x-col] <= 25:
                    adj_cells.append((x-col,y+row))
                if inBounds(x-col,y-row) and map[y-row][x-col] <= 25:
                    adj_cells.append((x-col,y-row))
        dist += 1

    nearestCell = 100
    for cell in range(len(adj_cells)):
        if EucDist(x,y,adj_cells[cell][0],adj_cells[cell][1]) + EucDist(adj_cells[cell][0],adj_cells[cell][1],end[0],end[1]) < nearestCell:
            nearestCell = EucDist(x,y,adj_cells[cell][0],adj_cells[cell][1]) + EucDist(adj_cells[cell][0],adj_cells[cell][1],end[0],end[1])
            idx = cell

    return [adj_cells[idx]]

def checkAdjacentCells(map, current_node, cells):
    children = []
    for new_pos in cells: # Adjacent squares
        # Get position of new node
        if len(cells) > 1:
            new_node = (current_node.position[0] + new_pos[0], current_node.position[1] + new_pos[1])
        else:
            new_node = (new_pos[0], new_pos[1])

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

    return children

def astar(map, start, end):
    # Adaptive threshold for exploration
    (x_start,y_start) = start
    (x_end,y_end) = end
    if Heuristic(x_start,y_start,x_end,y_end)**0.5 < 20:
        threshold = int(6*Heuristic(x_start,y_start,x_end,y_end)**0.5)
    else:
        threshold = int(5*Heuristic(x_start,y_start,x_end,y_end)**0.5)

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize lists
    living_list = [] # states that have been encountered, but possibly have unvisited next states
    dead_list = []   # states that cannot contribute to the search
    heapq.heapify(living_list)
    heapq.heapify(dead_list)

    # Add start node to the living list
    heapq.heappush(living_list,start_node)

    # Loop until we find goal
    while len(living_list) > 0:

        # Get the current node
        current_node = heapq.heappop(living_list)

        # Pop current node off the living list and add it to the dead list
        heapq.heappush(dead_list,current_node)

        # If we find goal, return reversed path
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        notFeasibleCell = True
        children = checkAdjacentCells(map, current_node, [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)])
        if not len(children):
            child = checkAdjacentCells(map, current_node, getAdjacentCells(map, current_node, end))
            heapq.heappush(living_list,child[0])
            continue


        # Loop through children
        for child in children:

            # Check if child is equal to child in the dead list
            if child in dead_list:
                continue

            # Calculate costs
            if map[child.position[1]][child.position[0]] != 0:
                child.g = current_node.g + 70
            else:
                child.g = current_node.g + 1

            if map[current_node.position[1]][current_node.position[0]] == 25:
                child.h = Heuristic(child.position[0], child.position[1], end_node.position[0], end_node.position[1])*1.5
            else:
                child.h = Heuristic(child.position[0], child.position[1], end_node.position[0], end_node.position[1])

            child.f = child.g + child.h

            # Check if child is already in the living list and check g-cost
            if living_list:
                if child in living_list or similarity_check(child, heapq.nlargest(1, living_list)[0]) > threshold and child.g > heapq.nlargest(1, living_list)[0].g:
                    continue

            heapq.heappush(living_list,child)

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

def checkForNextCell(path, gridmap, currentCell, nextCell):
    N = len(path)-1
    nextCellTemp = nextCell
    foundFree = False
    firstColl = False
    feaibleSpace = []
    while nextCellTemp <= N or foundFree:
        (x_temp, y_temp) = path[nextCellTemp]
        cells = raytrace(path[currentCell],path[nextCellTemp])
        feasible = True
        for (x,y) in cells:
            if gridmap[y][x] > 25:
                if not firstColl:
                    firstCollCell = nextCellTemp
                    firstColl = True
                feasible = False
                break
        if gridmap[y_temp][x_temp] == 0 and feasible:
            foundFree = True
            break
        elif feasible:
            feaibleSpace.append(nextCellTemp)

        nextCellTemp += 1

    if foundFree:
        currentCell = nextCellTemp
    elif len(feaibleSpace) != 0:
        currentCell = feaibleSpace[-1]
    else:
        currentCell = firstCollCell-1

    nextCell = currentCell

    return currentCell, nextCell

def pathsmoothing(path, gridmap):
    N = len(path)-1
    new_path = []
    new_path.append(path[0])

    currentCell = 0
    nextCell = 2
    while nextCell < N:

            ########################################################
            # Case 1:                                              #
            #   - If we are on dark look for next light            #
            #   else                                               #
            #   - Go to feasible area closest to goal              #
            ########################################################

            (xCurrent,yCurrent) = path[currentCell]
            (xNext,yNext) = path[nextCell]
            if gridmap[yCurrent][xCurrent] == 25:
                currentCell, nextCell = checkForNextCell(path, gridmap, currentCell, nextCell)

            ##########################################################################
            # Case 2:                                                                #
            #   - If we are on light:                                                #
            #       * if next is light --> check all remaning nodes and go to        #
            #         the last node in path that is light (with ray on light)        #
            #       else                                                             #
            #       * If next is dark:                                               #
            #           Look for next bright cell                                    #
            #           else:                                                        #
            #           Go to feasible area closest to goal                          #
            ##########################################################################
            # elif gridmap[yCurrent][xCurrent] == 0:
            #     if not gridmap[yNext][xNext] == 25:
            #         currentFree = nextCell
            #         for k in range(nextCell, N+1):
            #             (x_free_temp, y_free_temp) = path[k]
            #             if gridmap[y_free_temp][x_free_temp] == 0:
            #                 cells = raytrace(path[currentCell],path[k])
            #                 free = True
            #                 for (x,y) in cells:
            #                     if gridmap[y][x] > 0:
            #                         free = False
            #                         break
            #                 if free:
            #                     currentFree = k
            #         currentCell = currentFree
            #         nextCell = currentCell
            #     else:
            #         currentCell, nextCell = checkForNextCell(path, gridmap, currentCell, nextCell)
            elif gridmap[yCurrent][xCurrent] == 0:
                currentFree_flag = False
                for k in range(nextCell, N+1):
                    (x_free_temp, y_free_temp) = path[k]
                    if gridmap[y_free_temp][x_free_temp] == 0:
                        cells = raytrace(path[currentCell],path[k])
                        free = True
                        for (x,y) in cells:
                            if gridmap[y][x] > 25:
                                free = False
                                break
                        if free:
                            currentFree = k
                            currentFree_flag = True
                if currentFree_flag:
                    currentCell = currentFree
                    nextCell = currentCell
                elif gridmap[yNext][xNext] == 25:
                    currentCell, nextCell = checkForNextCell(path, gridmap, currentCell, nextCell)

            elif gridmap[yCurrent][xCurrent] > 25:
                (x_new,y_new) = path[currentCell]
                new_path.append((x_new,y_new))
                currentCell = currentCell + 1
                nextCell = currentCell + 1
                continue

            (x_new,y_new) = path[currentCell]
            new_path.append((x_new,y_new))
            nextCell += 2

    if not new_path[-1] == path[-1]:
        new_path.append(path[-1])

    return new_path

def smooth(path, alpha=0.9, beta=0.1, tolerance=0.000001):
    """
    Gradient ascent for smooting
        Params:
        alpha: weight of data
        beta: weight of smoothing
    """

    pathlist = []
    for (x,y) in path:
        pathlist.append([x,y])
    path = pathlist

    path_smooth = deepcopy(path)
    delta = tolerance

    while delta >= tolerance:
        delta = 0.0
        for i in range(1, len(path_smooth)-1):
            for j in range(len(path[0])):
                x_i = path[i][j]
                y_i, y_prev, y_next = path_smooth[i][j], path_smooth[i - 1][j], path_smooth[i + 1][j]

                y_i_store = y_i
                y_i += alpha * (x_i - y_i) + beta * (y_next + y_prev - (2 * y_i))
                path_smooth[i][j] = (y_i)

                delta += abs(y_i - y_i_store)

    new_path = []
    for row in path_smooth:
        new_path.append((row[0],row[1]))

    return new_path

class mainClass():
    def __init__(self):
        self.goal_sub = rospy.Subscriber("move_base_simple/goal",PoseStamped, self.goal_callback)
        self.start_sub = rospy.Subscriber("localisation/position", Vector3, self.start_callback)
        self.map_bounds_sub = rospy.Subscriber("/map_bounds", Point, self.map_bounds_callback)
        self.path_pub = rospy.Publisher("/aPath", Path, queue_size=10)
        self.error_pub = rospy.Publisher("/path_planner_error", Point, queue_size=10)
        self.goal_flag = False
        self.start_flag = False

    def goal_callback(self, data):
        self.goal = data
        self.goal_flag = True
        self.main()

    def start_callback(self, data):
        self.start_pos = data
        self.start_flag = True

    def map_bounds_callback(self, data):
        self.x_max = int(data.x)
        self.y_max = int(data.y)

    def inbounds(self, x, y):
        if x >= 0 and x < self.x_max and y >= 0 and y < self.y_max:
            return True
        else:
            return False

    def validgoal(self, map, goal):
        (x,y) = goal
        if not self.inbounds(x,y) or map[y][x] > 25:
            print("Goal is not reachable...")
            return False
        else:
            return True

    def calcDist(self, x_current, y_current, x_target, y_target):
        return (((x_target - x_current) ** 2) + ((y_target - y_current) ** 2))**0.5

    def checkNearestGoal(self, map, x, y):
        dist = 1
        cells = []
        while not len(cells):
            for row in range(dist):
                for col in range(dist):
                    if inBounds(x+col,y+row) and map[y+row][x+col] <= 25:
                        cells.append((x+col,y+row))
                    if inBounds(x+col,y-row) and map[y-row][x+col] <= 25:
                        cells.append((x+col,y-row))
                    if inBounds(x-col,y+row) and map[y+row][x-col] <= 25:
                        cells.append((x-col,y+row))
                    if inBounds(x-col,y-row) and map[y-row][x-col] <= 25:
                        cells.append((x-col,y-row))
            dist += 1

        nearestCell = 100
        for cell in range(len(cells)):
            if self.calcDist(x,y,cells[cell][0],cells[cell][1]) < nearestCell:
                nearestCell = self.calcDist(x,y,cells[cell][0],cells[cell][1])
                idx = cell

        return cells[idx]

    def main(self):
        if self.goal_flag:
            self.start_flag = False
            self.goal_flag = False
            map = Map()
            global x_max
            x_max = map.info.width
            global y_max
            y_max = map.info.height
            try:
                gridmap = np.array(map.info.data).reshape(map.info.height,map.info.width)
                gridmap = gridmap.tolist()

                # rospy.spin()
                # x1 = 0.2
                # y1 = 0.2
                # x1 = 1.32548177242
                # y1 = 3.38361263275

                x1 = self.start_pos.x
                y1 = self.start_pos.y

                x2 = self.goal.pose.position.x
                y2 = self.goal.pose.position.y

                map_idx_x1 = int((x1-map.info.origin_x)/map.info.resolution);
                map_idx_y1 = int((y1-map.info.origin_y)/map.info.resolution);

                map_idx_x2 = int((x2-map.info.origin_x)/map.info.resolution);
                map_idx_y2 = int((y2-map.info.origin_y)/map.info.resolution);

                start = (map_idx_x1, map_idx_y1)
                end   = (map_idx_x2, map_idx_y2)

                valid_goal = self.validgoal(gridmap,end)
                if not valid_goal:
                    raise Exception('Not valid goal')

                path = astar(gridmap, start, end)
                print("A* done")

                print("Smoothing path...")
                path = pathsmoothing(path,gridmap)
                # path = smooth(path)
                print(path)
                print("Path smoothed!")

                path_msg = Path()
                path_msg.header.frame_id = "base_map"

                for (x,y) in path:
                    pose = PoseStamped()
                    pose.pose.position.x = x*map.info.resolution + map.info.origin_x + map.info.resolution*0.5
                    pose.pose.position.y = y*map.info.resolution + map.info.origin_y + map.info.resolution*0.5
                    path_msg.poses.append(pose)

                self.path_pub.publish(path_msg)
            except:
                print("A* failed, finding nearest feasible cell")
                (map_idx_x2_new, map_idx_y2_new) = self.checkNearestGoal(gridmap, map_idx_x2, map_idx_y2)
                start = (map_idx_x1, map_idx_y1)
                end   = (map_idx_x2_new, map_idx_y2_new)

                path = astar(gridmap, start, end)
                print("A* done")

                print("Smoothing path...")
                path = pathsmoothing(path,gridmap)
                # path = smooth(path)
                print("Path smoothed!")

                path_msg = Path()
                path_msg.header.frame_id = "base_map"

                for (x,y) in path:
                    pose = PoseStamped()
                    pose.pose.position.x = x*map.info.resolution + map.info.origin_x + map.info.resolution*0.5
                    pose.pose.position.y = y*map.info.resolution + map.info.origin_y + map.info.resolution*0.5
                    path_msg.poses.append(pose)

                self.path_pub.publish(path_msg)

if __name__ == '__main__':
    rospy.init_node('ras_path_planning')
    try:
        mainClass()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
