import torch
from torch import nn
from time import time
import torchprof
import pytorch_lightning as pl
from lap_data import VideoDataset
from torch.utils.data import DataLoader
import wandb


class MobileBlock3d(nn.Module):

    def __init__(self, in_channels, out_channels, expand_channels, group_size=1):
        super().__init__()
        self.in_channels = in_channels
        self.out_channels = out_channels
        self.expand_channels = expand_channels

        if in_channels % group_size != 0 or out_channels % group_size != 0:
            group_size = 1

        self.conv_expand = nn.Conv3d(in_channels, expand_channels, (1, 1, 1), bias=False, groups=group_size)
        self.bn1 = nn.BatchNorm3d(expand_channels)
        self.depth_conv_xy = nn.Conv3d(expand_channels, expand_channels, (1, 3, 3),
                                       padding=(0, 1, 1), groups=expand_channels, bias=False)
        self.depth_conv_t = nn.Conv3d(expand_channels, expand_channels, (3, 1, 1),
                                      padding=(1, 0, 0), groups=expand_channels, bias=False)
        self.bn2 = nn.BatchNorm3d(expand_channels)
        self.conv_proj = nn.Conv3d(expand_channels, out_channels, (1, 1, 1), bias=False, groups=group_size)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):  # (batch_size, channels, frames, h, w)
        out = self.conv_expand(x)
        skip = out
        out = self.bn1(out)
        self.relu(out)
        out = self.depth_conv_xy(out)
        out = self.depth_conv_t(out)
        out = self.bn2(out)
        self.relu(out)
        out = out + skip
        out = self.conv_proj(out)
        return out


class GlobalAvgPool3d(nn.Module):

    def forward(self, x):  # (batch_size, channels, frames, h, w)
        return torch.mean(x, dim=(2, 3, 4))


class ControllerModel(nn.Module):

    def __init__(self):
        super().__init__()
        self.img_backbone = nn.Sequential(
            MobileBlock3d(3, 32, 32),
            nn.AvgPool3d((1, 2, 2)),
            MobileBlock3d(32, 32, 48, group_size=1),
            nn.AvgPool3d((1, 2, 2)),
            MobileBlock3d(32, 32, 48, group_size=1),
            nn.AvgPool3d((1, 2, 2)),
            MobileBlock3d(32, 32, 48, group_size=1),
            GlobalAvgPool3d(),
            nn.Linear(32, 64, bias=False),
        )
        self.control_backbone = nn.Sequential(
            nn.Conv1d(2, 32, (3,), padding=(1,), bias=False),
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.Conv1d(32, 32, (3,), padding=(1,), bias=False),
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.Conv1d(32, 8, (3,), padding=(1,), bias=False),
            nn.BatchNorm1d(8),
            nn.ReLU(),
            nn.Flatten(),
        )
        self.head = nn.Sequential(
            nn.Linear(64, 64, bias=False),
            nn.ReLU(),
            nn.Linear(64, 2)
        )

    def forward(self, imgs, controls):
        y1 = self.img_backbone(imgs)
        y2 = self.control_backbone(controls)
        out = y1 + y2
        out = self.head(out)
        return out


class TrainingModel(ControllerModel, pl.LightningModule):

    def __init__(self, lr=1e-3):
        super().__init__()
        self.lr = lr
        self.loss_func = nn.MSELoss()

    def configure_optimizers(self):
        return torch.optim.Adam(self.parameters(), lr=self.lr)

    def step(self, batch, step_type='train'):
        frames, t, steering, throttle, force, bat = batch
        steering, throttle = steering.to(torch.float), throttle.to(torch.float)
        imgs = torch.movedim(frames, -1, 1) / 255
        ctrl = torch.cat([steering[:, None], throttle[:, None]], 1)
        steering_pred, throttle_pred = self(imgs, ctrl).T
        steering_loss = self.loss_func(steering_pred, steering[:, -1])
        throttle_loss = self.loss_func(throttle_pred, throttle[:, -1])
        loss = steering_loss + throttle_loss
        wandb.log({
            step_type + '_throttle_loss': throttle_loss.item(),
            step_type + '_steering_loss': steering_loss.item(),
            step_type + '_loss': loss.item()
        })
        return loss

    def training_step(self, batch, i):
        return self.step(batch, 'train')

    def validation_step(self, batch, i):
        return self.step(batch, 'val')

    def on_epoch_end(self):
        torch.save(self.state_dict(), 'weights.pt')


if __name__ == '__main__':

    n_frames = 8
    res = (320, 240)
    batch_size = 6
    num_workers = 4
    epochs = 100
    lr = 1e-4

    train_ds = VideoDataset(split='train', n_frames=n_frames, res=res, clean=False)
    val_ds = VideoDataset(split='val', n_frames=n_frames, res=res, clean=False)
    train_dl = DataLoader(train_ds, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    val_dl = DataLoader(val_ds, batch_size=batch_size, shuffle=True, num_workers=num_workers)

    model = TrainingModel(lr=lr)
    trainer = pl.Trainer(gpus=[0], max_epochs=epochs)

    wandb.init(project='ai_rc', entity='grantmduffy')
    trainer.fit(model, train_dl, val_dl)
