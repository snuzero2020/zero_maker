import cv2
import numpy as np
import math

img = cv2.imread('/home/mu/catkin_ws/src/zero_maker/computer_vision/global_map_generator/img_sample/global_map_final.jpg')
drawTo = img.copy()
height = list(img.shape)[0]
width = list(img.shape)[1]
points = [[180, 254],[407, 254],[634, 254],[861, 254], [180, 487], [407, 487], [634, 487], [861, 487], [180, 720], [407, 720], [634, 720], [180, 953], [407, 953], [634, 953], [861, 953]]
pillars = []
Points = []
obstacle = []
input = np.array([])
transform_left = np.array([[1.68114392e+00, 9.90990102e+00, -7.13158948e+02], [-2.84368015e+00, 8.18311891e+00, 1.12497934e+03],[2.82940607e-05, 7.84113938e-03, 1.00000000e+00]])
transform_middle = np.array([[-3.12352150e+00, 3.53497363e+00, 1.51249690e+03], [-2.10094639e-03, -2.19766700e+00, 1.37084465e+03], [-4.22474667e-05, 6.79802622e-03, 1.00000000e+00]])
transform_right = np.array([[1.60224043e+00, -1.73514236e+00, 6.88165675e+02], [2.76754321e+00, 7.98408652e+00, -6.71916932e+02], [-3.77126242e-05, 7.67695952e-03, 1.00000000e+00]])



def generateCircle(radius, center):
    circle = []
    for i in range(radius + 1):
        for j in range(int(math.sqrt(radius * radius - i * i)) + 1):
            circle.append([center[0] + i, center[1] + j])
            circle.append([center[0] + i, center[1] - j])
            circle.append([center[0] - i, center[1] - j])
            circle.append([center[0] - i, center[1] + j])
    return circle

def generatePoints(input, transform):
    result = np.multiply(input, transform)
    np.reshape(result, (1,3))
    return result


#for i in range(height)[0:height:int(height/4)]:
    #for j in range(width)[0:width:int(width / 4)]:
        #points.append([i, j])
for i in range(len(points)):
    for pts in generateCircle(15, [points[i][0], points[i][1]]):
        if np.any(img[pts[0]][pts[1]]) and pts[0] * pts[1] >= 0 and pts[0] + pts[1] >= 0 and points[i] not in obstacle:
             obstacle.append(points[i])
        else:
            continue

for k in range(len(points)):
    if points[k] in obstacle:
        Points.append(1)
    else:
        Points.append(0)

print("이미지 크기")
print(height, width)
print('\n')
print("0은 장애물 없음, 1은 장애물이 15픽셀 반경 내에 존재")
print(Points)

cv2.waitKey(0)
cv2.destroyAllWindows()
