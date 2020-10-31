import cv2
import numpy as np
from matplotlib import pyplot as plt

img_middle = cv2.imread('/home/ayoung/Downloads/point_label_img_00005.jpg')
pts1 = np.float32([[88, 57], [320, 18], [551, 59], [320, 124]])
pts2 = np.float32([[1040, 900], [520, 1200], [0, 900], [520, 600]])

M = cv2.getPerspectiveTransform(pts1, pts2)
print("middle:")
print(M)

dst = cv2.warpPerspective(img_middle, M, (1040,1200))

plt.imshow(dst)
plt.show()

img_right = cv2.imread('/home/ayoung/Downloads/point_label_img_00006.jpg')
pts1 = np.float32([[87, 54], [315, 17], [540, 55], [313, 118]])
pts2 = np.float32([[520, 0], [1040, 300], [1040, 900], [520, 600]])

M = cv2.getPerspectiveTransform(pts1, pts2)
print("right:")
print(M)

dst = cv2.warpPerspective(img_right, M, (1040,1200))

plt.imshow(dst)
plt.show()

img_left = cv2.imread('/home/ayoung/Downloads/point_label_img_00005(2).jpg')
pts1 = np.float32([[100, 55], [324, 17], [551, 54], [327, 118]])
pts2 = np.float32([[0, 900], [0, 300], [520, 0], [520, 600]])

M = cv2.getPerspectiveTransform(pts1, pts2)
print("left:")
print(M)

dst = cv2.warpPerspective(img_left, M, (1040,1200))

plt.imshow(dst)
plt.show()