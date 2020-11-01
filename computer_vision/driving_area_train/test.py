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
import roslib

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--weight_path", '-w', type=str, default=roslib.packages.get_pkg_dir("driving_area_train") + "/checkpoints/ldln_ckpt_5.pth", help="Path to model weights")
    opt = parser.parse_args()

    USE_CUDA = torch.cuda.is_available()
    DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    test_path = opt.test_path
    BATCH_SIZE = opt.batch_size

    dataset = ListDataset(test_path)
    dataloader = torch.utils.data.DataLoader(dataset, batch_size= BATCH_SIZE, collate_fn=dataset.collate_fn)

    model = LaneNet().to(DEVICE)
    model.load_state_dict(torch.load(opt.weight_path))
    model.eval()
    totnum = 0
    accnum = 0

    start_time = time.time()
    while not rospy.is_shutdown():
        img_to = sample['img'].to(DEVICE)
        output = model(img_to)

        binary_seg = output['binary_seg']
        binary_seg_prob = binary_seg.detach().cpu().numpy()
        binary_seg_pred = np.argmax(binary_seg_prob, axis = 1)
        # print(binary_seg_pred.shape)

        binLabel = sample['binLabel'].detach().cpu().numpy()
        binLabel[binLabel != 0] = 1
        binLabel = np.squeeze(binLabel, axis = 1)
        # print(binLabel.shape)

        totnum += np.count_nonzero(binLabel)
        for b in range(len(binLabel)):
            for row in range(len(binLabel[0])):
                for col in range(len(binLabel[0][0])):
                    if binLabel[b][row][col] == 1 and binary_seg_pred[b][row][col] == 1:
                        accnum += 1

        print("Batch {0} completed.".format(i))
        
        # print(totnum)
        # print(accnum)
        # plt.imshow(binary_seg_pred, "gray")
        # plt.show()

    print("Duration: {0}, Accuracy: {1}".format((time.time()-start_time), accnum/totnum))
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    