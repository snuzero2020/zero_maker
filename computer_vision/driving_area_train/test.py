import argparse
import cv2
import torch
from scripts.model import *
import numpy as np
from torchvision.transforms import transforms
from scripts.utils import *
import time

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--img_path", '-i', type=str, default="/home/snuzero/catkin_ws/src/zero_maker/computer_vision/driving_area_train/data/img/0000.png", help="Path to demo img")
    parser.add_argument("--weight_path", '-w', type=str, help="Path to model weights")
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
    # pix_embedding = output['pix_embedding']
    # pix_embedding = pix_embedding.detach().cpu().numpy()
    # embedding = np.transpose(pix_embedding[0],(1,2,0))
    binary_seg = output['binary_seg']
    binary_seg_prob = binary_seg.detach().cpu().numpy()
    binary_seg_pred = np.argmax(binary_seg_prob, axis = 1)[0]

    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    seg_img = binary_seg_pred

    # print(time.time() - start_time)
    # embedding_time = time.time()

    # lane_seg_img = embedding_post_processing(embedding, binary_seg_pred, opt.band_width, 4)
    # color = np.array([[255, 125, 0], [0, 255, 0], [0, 0, 255], [0, 255, 255], [255, 0, 255]], dtype='uint8')
    
    # for i, lane_idx in enumerate(np.unique(lane_seg_img)):
    #     if lane_idx==0:
    #         continue
    #     seg_img[lane_seg_img == lane_idx] = color[i-1]
    print(img.shape)
    print(seg_img.shape)
    img = cv2.addWeighted(src1=seg_img, alpha=0.8, src2=img, beta=1., gamma=0.)

    print(time.time() - embedding_time)
    cv2.imshow("img", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    