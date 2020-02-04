#from statemachine_MS4 import *
from collections import namedtuple
import os
from statemachine_MS4 import Object
from geometry_msgs.msg import Point

#Object = namedtuple("Object", "clas pose conf")
Wall = namedtuple("Wall" , "x1 y1 x2 y2")
#Point = namedtuple("Point", "x y z")
saved_obj_file = "saved_objects.txt"
saved_walls_file = "saved_walls.txt"

def clean_storage():
        if os.path.exists(saved_obj_file):
                os.remove(saved_obj_file)
        if os.path.exists(saved_walls_file):
                os.remove(saved_walls_file)

def write_new_object(obj):
        with open(saved_obj_file, "a") as file:
                file.write(str(obj.clas) + " " + str(obj.pose.x) + " " + str(obj.pose.y) + " " + \
                        str(obj.pose.z) + " " + str(obj.conf) + "\n")
def write_new_wall(wall):
        with open(saved_walls_file, "a") as file:
                file.write(str(wall.x1) + " " + str(wall.y1) + " " + str(wall.x2) + " " + \
                        str(wall.y2) + "\n")


def write_objects(objects):
    for obj in objects:
        write_new_object(obj)

def write_walls(walls):
    for wall in walls:
        write_new_wall(wall)

def read_saved_objects():
    with open(saved_obj_file, "r") as file:
        lines = file.readlines()
        objects = []
        for line in lines:
                whitespacesplit = line.split()
                if len(whitespacesplit) > 4:
                        clas = int(whitespacesplit[0])
                        x = float(whitespacesplit[1])
                        y = float(whitespacesplit[2])
                        z = float(whitespacesplit[3])
                        p = Point()
                        p.x = x
                        p.y = y
                        p.z = z
                        conf = int(whitespacesplit[4])
                        obj = Object(clas, p, conf)
                        objects.append(obj)
        return condenseList(objects)

def condenseList(objects):
    checkedIds = []
    final = []
    for obj1 in objects:
        tempObj = obj1
        if tempObj.clas > 99:
            tempObj.clas = tempObj.clas - 100
        for obj2 in objects:
            if obj2.clas > 99:
                obj2.clas = obj2.clas - 100
            if obj1.id == obj2.id and obj1.clas != obj2.clas and obj2.clas != 15:
                tempObj = obj2
        #only append to list if id is not already checked.
        isAlreadyChecked = False
        for elem in checkedIds:
            if elem == tempObj.clas:
                isAlreadyChecked = True
        if not isAlreadyChecked:
            final.append(tempObj)
            checkedIds.append(tempObj.clas)
    print(final)
    return final

def read_saved_walls():
        with open(saved_walls_file, "r") as file:
            lines = file.readlines()
            walls = []
            for line in lines:
                    whitespacesplit = line.split()
                    if len(whitespacesplit) > 3:
                            x1 = whitespacesplit[0]
                            y1 = whitespacesplit[1]
                            x2 = whitespacesplit[2]
                            y2 = whitespacesplit[3]
                            wall = Wall(x1=x1, y1=y1, x2=x2, y2=y2)
                            walls.append(wall)
            return walls

# For testing
# if __name__ == "__main__":
    # obj = Object(5, Point(x=3, y=5), 95)
    # obj2 = Object(3, Point(x=4, y=5), 75)
    # obj3 = Object(4, Point(x=5, y=5), 75)
    # obj4 = Object(5, Point(x=6, y=5), 75)
    # obj5 = Object(6, Point(x=7, y=5), 75)

    # write_objects([obj, obj2, obj3, obj4, obj5])
    # for obj in read_objects():
    #     print(obj)
    #
    # wall1 = Wall(x1=1, y1=2, x2=3, y2=5)
    # wall2 = Wall(x1=2, y1=3, x2=4, y2=6)
    # wall3 = Wall(x1=3, y1=4, x2=5, y2=7)
    # wall4 = Wall(x1=4, y1=5, x2=6, y2=8)
    # write_walls([wall1, wall2, wall3, wall4])
    # for wall in read_saved_walls():
    #     print(wall)
