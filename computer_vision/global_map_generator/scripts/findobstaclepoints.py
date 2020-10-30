import cv2
import numpy as np
import math

img = cv2.imread('/home/mu/PycharmProjects/birdeye/venv/point_label_img_00005.jpg')
height = list(img.shape)[0]
width = list(img.shape)[1]
points = []

obstacle = []
print(height, width)

def generateCircle(radius, center):
    circle = []
    for i in range(radius+1):
        for j in range(int(math.sqrt(radius*radius-i*i))+1):
            circle.append([center[0]+i, center[1]+j])
            circle.append([center[0]+i, center[1]-j])
            circle.append([center[0]-i, center[1]-j])
            circle.append([center[0]-i, center[1]+j])
    return circle

for i in range(height)[0:height:int(height/4)]:
    for j in range(width)[0:width:int(width/4)]:
        points.append([i, j])
        print(points)
        for pts in generateCircle(15, [i, j]):
            if np.any(pts) and pts[0]*pts[1] >=0 and pts[0]+pts[1] >=0:
                obstacle.append(pts)

            else:
                continue

print("Points")
print(points)
print("Obstacles")
print(obstacle)
