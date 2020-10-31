import cv2
import numpy as np
import math

img = cv2.imread('/home/mu/PycharmProjects/birdeye/venv/global_map.jpg')
height = list(img.shape)[0]
width = list(img.shape)[1]
points = []
Points = []
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
            if np.any(img[pts[0]][pts[1]]) and pts[0]*pts[1] >=0 and pts[0]+pts[1] >=0 and [i,j] not in obstacle:
                obstacle.append([i,j])
            else:
                continue
for k in range(len(points)):
    if points[k] in obstacle:
        Points.append(1)
    else:
        Points.append(0)


print("Points")
print(points)
print('\n')

print("Obstacles")

print(obstacle)
print(len(obstacle))
print('\n')
print(Points)
