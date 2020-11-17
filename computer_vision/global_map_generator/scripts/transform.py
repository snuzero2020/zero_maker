import cv2
import numpy as np
from matplotlib import pyplot as plt

img_middle = cv2.imread('/home/ayoung/catkin_ws/src/zero_maker/computer_vision/global_map_generator/img_sample/cam1/point_label_img_00005.jpg')
pts1 = np.float32([[88, 58], [320, 18], [552, 58], [320, 124]])
pts2 = np.float32([[1040, 900], [520, 1200], [0, 900], [520, 600]])

M = cv2.getPerspectiveTransform(pts1, pts2)
print("middle:")
print(M)

dst_middle = cv2.warpPerspective(img_middle, M, (1040,1200))

plt.imshow(dst_middle)
plt.show()

img_right = cv2.imread('/home/ayoung/catkin_ws/src/zero_maker/computer_vision/global_map_generator/img_sample/cam2/point_label_img_00006.jpg')
pts1 = np.float32([[88, 53], [313, 15], [539, 53], [311, 116]])
pts2 = np.float32([[520, 0], [1040, 300], [1040, 900], [520, 600]])

M = cv2.getPerspectiveTransform(pts1, pts2)
print("right:")
print(M)

dst_right = cv2.warpPerspective(img_right, M, (1040,1200))

plt.imshow(dst_right)
plt.show()

img_left = cv2.imread('/home/ayoung/catkin_ws/src/zero_maker/computer_vision/global_map_generator/img_sample/cam3/point_label_img_00005.jpg')
pts1 = np.float32([[98, 54], [323, 16], [551, 54], [325, 118]])
pts2 = np.float32([[0, 900], [0, 300], [520, 0], [520, 600]])

M = cv2.getPerspectiveTransform(pts1, pts2)
print("left:")
print(M)

dst_left = cv2.warpPerspective(img_left, M, (1040,1200))

plt.imshow(dst_left)
plt.show()

bit_and_tmp = cv2.bitwise_and(dst_middle, dst_left)
bit_and = cv2.bitwise_and(dst_right, bit_and_tmp)

plt.imshow(bit_and)
plt.show()