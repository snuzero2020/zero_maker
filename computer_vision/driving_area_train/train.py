import argparse
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torchvision import transforms, datasets, models
import os
from scripts.model import *
from data_loader.dataset import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--epochs", type=int, default=100, help="number of epochs")
    parser.add_argument("--batch_size", type=int, default=6 , help="size of each image batch")
    parser.add_argument("--n_cpu", type=int, default=8, help="number of cpu threads to use during batch generation")
    parser.add_argument("--pretrained_weights", type=str, help="if specified starts from checkpoint model")
    parser.add_argument("--gradient_accumulations", type=int, default=2, help="number of gradient accums before step")
    parser.add_argument("--checkpoint_interval", type=int, default=1, help="interval between saving model weights")
    parser.add_argument("--train_path", type=str, help = "train.txt path")
    opt = parser.parse_args()

    USE_CUDA = torch.cuda.is_available()
    DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    os.makedirs("output", exist_ok=True)
    os.makedirs("checkpoints", exist_ok=True)

    EPOCHS = opt.epochs
    BATCH_SIZE = opt.batch_size
    N_CPU = opt.n_cpu
    train_path = opt.train_path

    model = LaneNet().to(DEVICE)
    optimizer = torch.optim.Adam(model.parameters())

    if opt.pretrained_weights:
        model.load_state_dict(torch.load(opt.pretrained_weights))

    dataset = ListDataset(train_path)
    dataloader = torch.utils.data.DataLoader(
        dataset,
        batch_size= BATCH_SIZE,
        shuffle= True,
        num_workers= N_CPU,
        pin_memory= True,
        collate_fn= dataset.collate_fn
    )

    for epoch in range(opt.epochs):
        model.train()
        for batch_idx, sample in enumerate(dataloader):
            img = Variable(sample['img'].to(DEVICE))
            segLabel = Variable(sample['segLabel'].to(DEVICE), requires_grad= False)

            batches_done = len(dataloader) * epoch + batch_idx

            output = model(img, segLabel)
            # pix_embedding = output['pix_embedding']
            # binary_seg = output['binary_seg_ret']
            cluster_loss = output['cluster_loss']
            seg_loss = output['seg_loss']
            total_loss = output['total_loss']

            total_loss.backward()

            if batches_done % opt.gradient_accumulations:
                optimizer.step()
                optimizer.zero_grad()

            print('[{}][{}] Cluster loss : {:.4f} Seg loss: {:.4f} Total loss: {:.4f}'.format(epoch, batch_idx, cluster_loss, seg_loss, total_loss))
        if epoch % opt.checkpoint_interval == 0:
            torch.save(model.state_dict(), f"checkpoints/ldln_ckpt_%d.pth" % epoch)



# return {
#             'pix_embedding' : pix_embedding,
#             'binary_seg' : binary_seg_ret,
#             'cluster_loss' : cluster_loss,
#             'seg_loss' : seg_loss,
#             'total_loss' : total_loss
#         }

