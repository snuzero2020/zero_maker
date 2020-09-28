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
    # parser.add_argument("--img_path", '-i', type=str, default="/home/ayoung/catkin_ws/src/zero_maker/computer_vision/data/img/0001.png", help="Path to demo img")
    parser.add_argument("--weight_path", '-w', type=str, default="/home/ayoung/catkin_ws/src/zero_maker/computer_vision/checkpoints/ldln_ckpt_99.pth", help="Path to model weights")
    parser.add_argument("--band_width", '-b', type=float, default=1.5, help="Value of delta_v")
    parser.add_argument("--visualize", '-v', action="store_true", default=False, help="Visualize the result")
    parser.add_argument("--test_path", type=str, default="/home/ayoung/catkin_ws/src/zero_maker/computer_vision/test_data/test.txt", help = "test.txt path")
    opt = parser.parse_args()

    USE_CUDA = torch.cuda.is_available()
    DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    test_path = opt.test_path

    dataset = ListDataset(test_path)
    dataloader = torch.utils.data.DataLoader(dataset, collate_fn=dataset.collate_fn)

    # img = cv2.imread(opt.img_path)
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # img = cv2.resize(img, (640, 480))
    # img_to = transforms.ToTensor()(img)
    # img_to = torch.stack([img_to]).to(DEVICE)

    model = LaneNet().to(DEVICE)
    model.load_state_dict(torch.load(opt.weight_path))
    model.eval()

    start_time = time.time()
    for sample in dataloader:
        img_to = sample['img'].to(DEVICE)
        # binLabel = Variable(sample['binLabel'].to(DEVICE), requires_grad= False)
        output = model(img_to)
        binary_seg = output['binary_seg']
        binary_seg_prob = binary_seg.detach().cpu().numpy()
        # print(np.count_nonzero(binary_seg_prob))
        binary_seg_pred = np.argmax(binary_seg_prob, axis = 1)[0]
        # print(np.count_nonzero(binary_seg_pred))
        # binary_seg_pred = np.array(binary_seg_pred, dtype = np.uint8)
        # print(np.count_nonzero(binary_seg_pred))
        binLabel = sample['binLabel'].detach().cpu().numpy()
        # print(np.count_nonzero(binLabel))
        binLabel = binLabel[0][0]
        print(np.count_nonzero(binLabel))
        print(binLabel)
        # binLabel = np.array(binLabel, dtype = np.uint8)
        print(np.count_nonzero(binLabel))
        # binLabel = np.stack((binLabel), axis = 2)
        # binary_seg_pred = np.stack((binary_seg_pred), axis = 2)
        print(binLabel.shape)
        print(binary_seg_pred.shape)
        # plt.imshow(binLabel, "gray")
        plt.imshow(binary_seg_pred, "gray")
        plt.show()
    
    # img = cv2.addWeighted(src1=seg_img, alpha=0.8, src2=img, beta=1., gamma=0.)

    # print(time.time() - embedding_time)
    # cv2.imshow("binary_seg_pred", binary_seg_pred)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    