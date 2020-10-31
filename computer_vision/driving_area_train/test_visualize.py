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

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--img_path", '-i', type=str, default="/home/sunho/catkin_ws/src/zero_maker/computer_vision/driving_area_train/img/color/camera_rgb_9999.jpg", help="Path to demo img")
    parser.add_argument("--weight_path", '-w', default = "/home/sunho/catkin_ws/src/zero_maker/computer_vision/driving_area_train/checkpoints/ldln_ckpt_5.pth", type=str, help="Path to model weights")
    parser.add_argument("--band_width", '-b', type=float, default=1.5, help="Value of delta_v")
    parser.add_argument("--visualize", '-v', action="store_true", default=False, help="Visualize the result")
    opt = parser.parse_args()

    USE_CUDA = torch.cuda.is_available()
    DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    img = cv2.imread(opt.img_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (640, 480))
    img_to = transforms.ToTensor()(img)
    img_to = torch.stack([img_to]).to(DEVICE)

    model = LaneNet().to(DEVICE)
    model.load_state_dict(torch.load(opt.weight_path))
    model.eval()

    start_time = time.time()
    output = model(img_to)
    binary_seg = output['binary_seg']
    binary_seg_prob = binary_seg.detach().cpu().numpy()
    binary_seg_pred = np.argmax(binary_seg_prob, axis = 1)[0]

    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    seg_img = np.zeros_like(img)
    seg_img[binary_seg_pred == 1] = 255

    cv2.imshow("img", seg_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()