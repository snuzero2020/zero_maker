import glob
import random
import os
import sys
import numpy as np
from PIL import Image
from torchvision.transforms import transforms
from torch.utils.data import Dataset
import torch
import cv2

class ListDataset(Dataset):
    def __init__(self, list_path):
        self.batch_count = 0
        self.img_files_ori = []
        self.img_files_bin = []
        self.img_files_ins = []
        with open(list_path, "r") as file:
            lst = file.readlines()
            for line in lst:
                tmp = line.split()
                self.img_files_ori = self.img_files_ori + [tmp[0]]
                self.img_files_bin = self.img_files_bin + [tmp[1]]
                self.img_files_ins = self.img_files_ins + [tmp[2]]
    
    def __getitem__(self, index):
        img_path = self.img_files_ori[index % len(self.img_files_ori)].rstrip()
        img_ins_path = self.img_files_ins[index % len(self.img_files_ins)].rstrip()

        img = cv2.imread(img_path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640, 480))
        img = transforms.ToTensor()(img)
        
        segLabel = cv2.resize(cv2.imread(img_ins_path), (640, 480))[:,:,0]
        segLabel = transforms.ToTensor()(segLabel)

        sample = {
            'img' : img,
            'segLabel' : segLabel,
            'img_name' : img_path
        }
        return sample
    
    def collate_fn(self, batch):
        img = torch.stack([b['img'] for b in batch])
        segLabel = torch.stack([b['segLabel'] for b in batch])
        samples = {
            'img' : img,
            'segLabel' : segLabel,
            'img_name' : [x['img_name'] for x in batch]
        }
        return samples
    
    def __len__(self):
        return len(self.img_files_ori)



# img_files_ori = []
# img_files_bin = []
# img_files_ins = []
# list_path = "/home/snuzero/VISION/LDLN/lane_detection_data/train.txt"
# with open(list_path, "r") as file:
#     lst = file.readlines()
#     for line in lst:
#         tmp = line.split()
#         img_files_ori = img_files_ori + [tmp[0]]
#         img_files_bin = img_files_bin + [tmp[1]]
#         img_files_ins = img_files_ins + [tmp[2]]
#     print(img_files_ori)
#     print(img_files_bin)
#     print(img_files_ins)