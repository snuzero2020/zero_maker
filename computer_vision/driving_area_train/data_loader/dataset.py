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
        with open(list_path, "r") as file:
            lst = file.readlines()
            for line in lst:
                tmp = line.split()
                self.img_files_ori = self.img_files_ori + [tmp[0]]
                self.img_files_bin = self.img_files_bin + [tmp[1]]
    
    def __getitem__(self, index):
        img_path = self.img_files_ori[index % len(self.img_files_ori)].rstrip()
        img_bin_path = self.img_files_bin[index % len(self.img_files_bin)].rstrip()

        img = cv2.imread(img_path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640, 480))
        img = transforms.ToTensor()(img)

        binLabel = cv2.resize(cv2.imread(img_bin_path), (640, 480))[:,:,0]
        binLabel = transforms.ToTensor()(binLabel)

        sample = {
            'img' : img,
            'binLabel' : binLabel,
            'img_name' : img_path
        }
        return sample
    
    def collate_fn(self, batch):
        img = torch.stack([b['img'] for b in batch])
        binLabel = torch.stack([b['binLabel'] for b in batch])
        samples = {
            'img' : img,
            'binLabel' : binLabel,
            'img_name' : [x['img_name'] for x in batch]
        }
        return samples
    
    def __len__(self):
        return len(self.img_files_ori)