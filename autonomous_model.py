import torch
from torch import nn
import pytorch_lightning as pl
import torchvision
from lap_data import VideoDataset
from torch.utils.data import DataLoader
import wandb


class LambdaLayer(torch.nn.Module):

    def __init__(self, l):
        super().__init__()
        self.l = l

    def forward(self, x):
        return self.l(x)


class Model(pl.LightningModule):

    def __init__(self, lr=1e-3):
        super().__init__()
        self.backbone = torchvision.models.video.r2plus1d_18(pretrained=False)
        self.backbone.fc = nn.Linear(512, 2)
        self.loss_func = nn.MSELoss()
        self.lr = lr

    def forward(self, x):
        return self.backbone(x)

    def configure_optimizers(self):
        return torch.optim.Adam(self.parameters(), lr=self.lr)

    def step(self, batch, type='train'):
        frames, t, steering, throttle, force, bat = batch
        steering, throttle = steering.to(torch.float), throttle.to(torch.float)
        x = torch.movedim(frames, -1, 1) / 255
        steering_pred, throttle_pred = self(x).T
        steering_loss = self.loss_func(steering_pred, steering[:, -1])
        throttle_loss = self.loss_func(throttle_pred, throttle[:, -1])
        loss = steering_loss + throttle_loss
        wandb.log({
            type + '_throttle_loss': throttle_loss.item(),
            type + '_steering_loss': steering_loss.item(),
            type + '_loss': loss.item()
        })
        return loss

    def training_step(self, batch, i):
        return self.step(batch, type='train')

    def validation_step(self, batch, i):
        return self.step(batch, type='val')

    def on_epoch_end(self):
        torch.save(self.state_dict(), 'weights.pt')


if __name__ == '__main__':

    n_frames = 4
    res = (112, 112)
    batch_size = 16
    num_workers = 4
    epochs = 100
    lr = 1e-4

    train_ds = VideoDataset(split='train', n_frames=n_frames, res=res)
    val_ds = VideoDataset(split='val', n_frames=n_frames, res=res)
    train_dl = DataLoader(train_ds, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    val_dl = DataLoader(val_ds, batch_size=batch_size, shuffle=True, num_workers=num_workers)

    model = Model(lr=lr)
    trainer = pl.Trainer(gpus=[0], max_epochs=epochs)

    wandb.init(project='ai_rc', entity='grantmduffy')
    trainer.fit(model, train_dl, val_dl)

