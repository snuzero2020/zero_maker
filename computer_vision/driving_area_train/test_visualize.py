import argparse
import cv2
import torch
from scripts.model import *
import numpy as np
from torchvision.transforms import transforms
from scripts.utils import *
from data_loader.dataset import *
import time
import matplotlib.pyplot as plt
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters
from driving_area_train.msg import seg_msg
from rospy.numpy_msg import numpy_msg
import roslib

class image_publisher:
    def __init__(self):
        self.USE_CUDA = torch.cuda.is_available()
        self.DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        image_middle_sub = message_filters.Subscriber("/camera1/color/image_raw", Image)
        image_right_sub = message_filters.Subscriber("/camera2/color/image_raw", Image)
        image_left_sub = message_filters.Subscriber("/camera3/color/image_raw", Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([image_middle_sub, image_right_sub, image_left_sub], 1, 1, allow_headerless=True)
        self.ts.registerCallback(self.callback)
        self.seg_pub = rospy.Publisher('/seg_topic', numpy_msg(seg_msg), queue_size = 10)

        rospy.loginfo("initialized")
    def callback(self, middle_image, right_image, left_image):
        bridge = CvBridge()

        USE_CUDA = torch.cuda.is_available()
        DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        seg_msg_pub = seg_msg()

        model = LaneNet().to(DEVICE)
        model.load_state_dict(torch.load(roslib.packages.get_pkg_dir("driving_area_train") + "/checkpoints/ldln_ckpt_5.pth"))
        model.eval()

        image_middle = bridge.imgmsg_to_cv2(middle_image, "bgr8")
        image_right = bridge.imgmsg_to_cv2(right_image, "bgr8")
        image_left = bridge.imgmsg_to_cv2(left_image, "bgr8")

        image_middle = cv2.cvtColor(image_middle, cv2.COLOR_BGR2RGB)
        image_middle = cv2.resize(image_middle, (640, 480))
        image_middle_to = transforms.ToTensor()(image_middle)

        image_right = cv2.cvtColor(image_right, cv2.COLOR_BGR2RGB)
        image_right = cv2.resize(image_right, (640, 480))
        image_right_to = transforms.ToTensor()(image_right)

        image_left = cv2.cvtColor(image_left, cv2.COLOR_BGR2RGB)
        image_left = cv2.resize(image_left, (640, 480))
        image_left_to = transforms.ToTensor()(image_left)

        img_input = torch.stack([image_middle_to, image_right_to, image_left_to], dim = 0).to(DEVICE)

        output = model(img_input)

        binary_seg = output['binary_seg']
        binary_seg_prob = binary_seg.detach().cpu().numpy()
        seg_msg_pub = binary_seg_prob.flatten()

        self.seg_pub.publish(seg_msg_pub)

if __name__ == "__main__":
    rospy.init_node('image_publisher', anonymous=True)
    img_pub = image_publisher()
    rospy.spin()