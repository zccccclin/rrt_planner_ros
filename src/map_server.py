#!/usr/bin/env python3
import pathlib
import rospy
import cv2
from pathlib import Path
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

rospy.init_node("map_server",anonymous=True)

map_msg = OccupancyGrid()
map_msg.header.frame_id = 'map'
resolution = 0.1

drawing=False # true if mouse is pressed
mode=True

def nothing(x):
    pass

# mouse callback function
def paint_draw(event,former_x,former_y,flags,param):
    global current_former_x,current_former_y,drawing, mode
    size = cv2.getTrackbarPos("Brush Size","Map Creation")
    erase = cv2.getTrackbarPos("Eraser On Off","Map Creation")
    color = (0,0,0)
    if erase:
        color = (255,255,255)
    if event==cv2.EVENT_LBUTTONDOWN:
        drawing=True
        current_former_x,current_former_y=former_x,former_y

    elif event==cv2.EVENT_MOUSEMOVE:
        if drawing==True:
            if mode==True:
                cv2.line(image,(current_former_x,current_former_y),(former_x,former_y),color,size)
                current_former_x = former_x
                current_former_y = former_y
    elif event==cv2.EVENT_LBUTTONUP:
        drawing=False
        if mode==True:
            cv2.line(image,(current_former_x,current_former_y),(former_x,former_y),color,size)
            current_former_x = former_x
            current_former_y = former_y
    return former_x,former_y


pub_init = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size=1)
pub_goal = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
pub = rospy.Publisher("/map",OccupancyGrid,queue_size=10)

if __name__ == '__main__':
    map_msg.info.resolution = resolution
    map_msg.data = []
    map_published = False
    input_received = False
    while not rospy.is_shutdown():
        if not map_published:
            custom_map = int(input("Load map image (1) or Build map image (2): "))
        if custom_map == 1:
            if not map_published:
                img_path = str(input("Enter map name (eg. map1.png): " ))
                subdir = "resources"
                parent = Path(__file__).parent.parent
                img_path = str(Path(parent,subdir,img_path))
                img = cv2.imread(img_path)
                h,w = img.shape[:2]
                map_msg.info.width = w
                map_msg.info.height = h
                imagem = np.invert(img)
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
                img = np.invert(cv2.dilate(imagem, kernel, iterations=1))
                img = np.flip(np.rot90(np.rot90(img)),1)
                for height in range(0,h):
                    for width in range(0,w):
                        if img[height,width].all() == 0:
                            map_msg.data.append(100)
                        else:
                            map_msg.data.append(0)
                pub.publish(map_msg)
                map_published = True
            start_x = int(input("Enter Initial X index (<=Width): "))
            start_y = int(input("Enter Initial Y index (<=Height): "))
            end_x = int(input("Enter Goal X index: (<=Width): "))
            end_y = int(input("Enter Goal Y index (<=Height)): "))
            start = PoseWithCovarianceStamped()
            start.pose.pose.position.x = start_x*resolution
            start.pose.pose.position.y = start_y*resolution
            pub_init.publish(start)
            end = PoseStamped()
            end.pose.position.x = end_x*resolution
            end.pose.position.y = end_y*resolution
            pub_goal.publish(end)
            
        elif custom_map == 2:
            if not input_received:
                w = int(input("Enter Width of custom map: "))
                h = int(input("Enter Height of custom map: "))
                input_received = True
                map_msg.info.width = w
                map_msg.info.height = h
                image = np.zeros((h,w,3),np.uint8)
                image.fill(255)
                cv2.namedWindow("Map Creation")
                cv2.createTrackbar("Brush Size","Map Creation",5,20,nothing)
                cv2.createTrackbar("Eraser On Off","Map Creation",0,1,nothing)
                cv2.setMouseCallback('Map Creation',paint_draw)
                while(not map_published):
                    cv2.imshow('Map Creation',image)
                    k=cv2.waitKey(1)& 0xFF
                    if k==27: #Escape KEY
                        for height in range(0,len(image)):
                            for width in range(0,len(image[height])):
                                if image[height][width].all() == 0:
                                    map_msg.data.append(100)
                                else:
                                    map_msg.data.append(0)
                        pub.publish(map_msg)
                        map_published = True
                cv2.destroyAllWindows()
            start_x = int(input("Enter Initial X index (<=Width): "))
            start_y = int(input("Enter Initial Y index (<=Height): "))
            end_x = int(input("Enter Goal X index: (<=Width): "))
            end_y = int(input("Enter Goal Y index (<=Height)): "))
            start = PoseWithCovarianceStamped()
            start.pose.pose.position.x = start_x*resolution
            start.pose.pose.position.y = start_y*resolution
            pub_init.publish(start)
            end = PoseStamped()
            end.pose.position.x = end_x*resolution
            end.pose.position.y = end_y*resolution
            pub_goal.publish(end)
        else:
            print("Invalid Input. Enter only 1 or 2")

