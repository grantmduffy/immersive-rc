from pathlib import Path
import numpy as np
import cv2
import pandas as pd
from torch.utils.data import Dataset
import wandb


def get_video_props(p: Path):
    cap = cv2.VideoCapture(str(p))
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    n = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    return int(w), int(h), int(n), fps


def crop_and_scale(img, res=(640, 480), interpolation=cv2.INTER_CUBIC):
    in_res = (img.shape[1], img.shape[0])
    r_in = in_res[0] / in_res[1]
    r_out = res[0] / res[1]

    if r_in > r_out:
        padding = int(round((in_res[0] - r_out * in_res[1]) / 2))
        img = img[:, padding:-padding]
    if r_in < r_out:
        padding = int(round((in_res[1] - in_res[0] / r_out) / 2))
        img = img[padding:-padding]

    img = cv2.resize(img, res, interpolation=interpolation)

    return img


class VideoDataset(Dataset):

    def __init__(self, split=None, n_frames=4, fps=70, res=(112, 112), clean=True,
                 data_path=Path(__file__).parent / 'data' / 'lap_data', val_split=0.1,
                 min_lap=190, max_lap=500):
        self.data_path = data_path
        self.split = split
        self.n_frames = n_frames
        self.fps = fps
        self.res = res
        self.min_lap = min_lap
        self.max_lap = max_lap
        index_path = data_path / 'index.csv'
        if index_path.exists() and not clean:
            self.index = pd.read_csv(index_path)
        else:
            self.index = pd.DataFrame()
            laps = {int(p.name[3:-4]) for p in data_path.iterdir()
                    if 'lap' in p.name and '.csv' in p.name}
            laps = laps.intersection({int(p.name[3:-4]) for p in data_path.iterdir()
                                      if 'lap' in p.name and '.avi' in p.name})
            self.index['lap'] = sorted(laps)
            self.index['split'] = np.random.choice(['train', 'val'], len(self.index), p=(1 - val_split, val_split))
            self.index['length'] = [get_video_props(data_path / f'lap{i}.avi')[2] for i in self.index['lap']]
            self.index = self.index[self.index['length'] != 0]
            self.index = self.index[self.index['length'] >= min_lap]
            self.index = self.index[self.index['length'] <= max_lap]
            self.index.to_csv(index_path, index=False)
        if split is not None:
            self.index = self.index[self.index['split'] == split]

    def __getitem__(self, item):
        lap_i, vid_len = self.index.iloc[item][['lap', 'length']]
        frames = []

        cap = cv2.VideoCapture(str(self.data_path / f'lap{lap_i}.avi'))
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            if self.res is not None:
                frame = crop_and_scale(frame, self.res)
            frames.append(frame)
        cap.release()
        frames = np.array(frames)

        df = pd.read_csv(self.data_path / f'lap{lap_i}.csv')
        t, steering, throttle, force, bat = np.array(df[['Time', 'Steering', 'Throttle', 'Force Feedback', 'Battery Level']]).T

        # TODO: Sample time and interpolate to simulate different fps
        # t_start = np.random.uniform(t.min(), t.max() - self.n_frames / self.fps)
        # t_sample = np.arange(self.n_frames) / self.fps + t_start
        # print(df)

        si = np.random.randint(0, len(t) - self.n_frames)
        frames = frames[si: si + self.n_frames]
        t = t[si: si + self.n_frames]
        steering = steering[si: si + self.n_frames]
        throttle = throttle[si: si + self.n_frames]
        force = force[si: si + self.n_frames]
        bat = bat[si: si + self.n_frames]

        return frames, t, steering, throttle, force, bat

    def __len__(self):
        return len(self.index)


if __name__ == '__main__':
    ds = VideoDataset(split='val')
    frames, t, s, thr, fr, bat = ds[0]
    print(frames.shape)
