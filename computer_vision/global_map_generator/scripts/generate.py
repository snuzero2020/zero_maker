import numpy as np
import cv2

img = cv2.imread('/home/mu/PycharmProjects/birdeye/venv/global_map.jpg', cv2.IMREAD_UNCHANGED)
(b, g, r) = img[0, 0]
print("Pixel at (0, 0) - Red: {}, Green: {}, Blue: {}".format(r,g, b))

pt1=np.array([[0,450],[260,600],[0,600]],np.int32)
pt2=np.array([[520,450],[260,600],[520,600]],np.int32)
pt3=np.array([[0,150],[260,0],[0,0]],np.int32)
pt4=np.array([[520,150],[260,0],[520,0]],np.int32)

cv2.fillConvexPoly(img,pt1,(255,255,255))
cv2.fillConvexPoly(img,pt2,(255,255,255))
cv2.fillConvexPoly(img,pt3,(255,255,255))
cv2.fillConvexPoly(img,pt4,(255,255,255))


cv2.imshow("fillConvexPoly", img)
cv2.imwrite('/home/mu/PycharmProjects/birdeye/venv/global_map_final.jpg', img)
cv2.waitKey(0)
cv2.destroyAllWindows()