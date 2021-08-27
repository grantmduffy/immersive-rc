from logiwheel import LogiWheel
from time import sleep, time
import struct
import serial
from threading import Thread
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import cv2
from pathlib import Path
import pandas as pd
import torch
from autonomous_model import Model
from grants_model import ControllerModel as GrantsModel
from lap_data import crop_and_scale

import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

display_res = (1280, 960)

ser_port = 'COM4'
baudrate = 115200
delay_time = 0.01
in_header_format = '4s'
in_time_format = 'L8x'
in_car_format = 'fHHf'

battery_scale = 412.63965
steering_offset = 0.0
steering_scale = 1.3

plot_n = 100
battery_history = np.zeros(plot_n)
force_history = np.zeros(plot_n)
throttle_gamma = 0.6

plt.figure()
plt.subplot(211)
bat_line, = plt.plot(battery_history)
plt.plot([0, plot_n - 1], [7.4, 7.4], 'r')
plt.ylim(7.0, 9.0)
plt.subplot(212)
force_line, = plt.plot(force_history)
plt.ylim(-1.0, 1.0)

d = dict(
    steering=0.0,
    throttle=0.0,
    force_feedback=0.0,
    actual_steering_pos=0.0,
    battery_level=0.0,
    steering_motor_force=0.0,
    buttons=[]
)
run = True
packets_sent = 0
packets_received = 0

# device = 'cuda:0'
device = 'cpu'
buffer_res = (112, 112)
buffer_size = 8
frame_buffer = torch.zeros((1, 3, buffer_size, buffer_res[1], buffer_res[0]), device=device, dtype=torch.float)
controls_buffer = torch.zeros((1, 2, buffer_size), device=device, dtype=torch.float)

lap_data_path = Path.cwd() / 'data' / 'lap_data'
if not lap_data_path.is_dir():
    lap_data_path.mkdir()
lap_i = max([int(p.name.replace('lap', '').replace('.avi', ''))
             for p in lap_data_path.iterdir() if '.avi' in p.name] + [-1])
new_lap = False
lap_start_t = time()
lap_data_columns = ['Time', 'Steering', 'Throttle', 'Force Feedback', 'Battery Level']
lap_data = []
video_cap = None
current_frame = np.zeros((480, 640, 3), np.uint8)

autonomous_mode = False
autonomous_steering = 0.0
autonomous_throttle = 0.0


def update(data):
    global d, steering_offset, steering_scale, lap_start_t, lap_i, new_lap, autonomous_mode
    d.update(data)
    if 'RSB' in d['buttons']:
        steering_offset = -d['steering']
    if 'LSB' in d['buttons']:
        steering_scale = 1 / abs(d['steering'])
    if 'SHIFT_UP' in d['buttons']:
        t = time()
        if t - lap_start_t > 2.0:
            print(lap_i, t - lap_start_t)
            lap_start_t = t
            lap_i += 1
            new_lap = True
    if 'A' in d['buttons']:
        autonomous_mode = True
    if 'B' in d['buttons']:
        autonomous_mode = False


def ser_thread():
    print('Serial thread starting')
    global d, run, packets_sent, packets_received, new_lap, video_cap, lap_data, lap_start_t, lap_i
    with serial.Serial(ser_port, baudrate) as ser:
        sleep(2)
        while run:
            if not autonomous_mode:
                steering_input = steering_scale * d['steering'] - steering_offset + 0.5
                throttle_input = d['throttle'] ** throttle_gamma
            else:
                steering_input = autonomous_steering
                throttle_input = autonomous_throttle

            # Update buffers
            frame_buffer[:] = torch.roll(frame_buffer, 1, 2)
            frame_buffer[0, :, 0] = torch.from_numpy(np.moveaxis(
                crop_and_scale(current_frame, (112, 112)), -1, 0) / 255.0).to(torch.float)
            controls_buffer[:] = torch.roll(controls_buffer, 1, 2)
            controls_buffer[0, 0, 0] = steering_input
            controls_buffer[0, 1, 0] = throttle_input

            ser.write(struct.pack('ff', steering_input, throttle_input))
            packets_sent += 1
            if ser.in_waiting >= struct.calcsize(in_header_format):
                header = struct.unpack(in_header_format, ser.read(struct.calcsize(in_header_format)))[0]
                if header == b'TIME':
                    t, = struct.unpack(in_time_format, ser.read(struct.calcsize(in_time_format)))
                    print(lap_i, t / 1000)
                    lap_start_t = time()
                    new_lap = True
                else:
                    d['force_feedback'], d['actual_steering_pos'], battery_level_raw, d['steering_force'] \
                        = struct.unpack(in_car_format, ser.read(struct.calcsize(in_car_format)))
                    d['battery_level'] = battery_level_raw / battery_scale
                    battery_history[:-1] = battery_history[1:]
                    battery_history[-1] = d['battery_level']
                    force_history[:-1] = force_history[1:]
                    force_history[-1] = d['force_feedback']
                ser.flushInput()
                packets_received += 1

                if new_lap:
                    if video_cap is not None:
                        video_cap.release()
                    new_lap = False
                    video_cap = cv2.VideoWriter(
                        str(lap_data_path / f'lap{lap_i + 1}.avi'),
                        cv2.VideoWriter_fourcc(*'XVID'),
                        75.0,
                        (640, 480)
                    )
                    pd.DataFrame(data=lap_data, columns=lap_data_columns).to_csv(lap_data_path / f'lap{lap_i}.csv', index=False)
                    lap_data = []
                if video_cap is not None:
                    video_cap.write(current_frame)
                    lap_data.append([time() - lap_start_t, steering_input,
                                     throttle_input, d['force_feedback'], d['battery_level']])
            sleep(delay_time)


def camera_thread():
    global run, current_frame
    cap = cv2.VideoCapture(1)
    while run:
        ret, frame = cap.read(current_frame)
        if not ret:
            continue
        display_frame = cv2.resize(frame, display_res)
        cv2.imshow('FPV Camera', display_frame)
        cv2.waitKey(1)
        if cv2.getWindowProperty('FPV Camera', 0) != 0.0:
            run = False


def wheel_thread():
    print('Wheel thread starting')
    global d, run
    with LogiWheel() as wheel:
        wheel.on_input = update
        wheel.reset_all_effects()
        sleep(5)
        const_id = wheel.constant_effect(0)
        while run:
            wheel.constant_effect(-d['force_feedback'], const_id)
            sleep(0.1)


def inference_thread():
    global frame_buffer, controls_buffer, autonomous_throttle, autonomous_steering
    # model = Model().to(device)
    model = GrantsModel().to(device)
    print(model.load_state_dict(torch.load('weights.pt')))
    while run:
        # preds = model(frame_buffer)
        preds = model(frame_buffer, controls_buffer)
        autonomous_steering = min(max(preds[0, 0].item(), -1), 1)
        autonomous_throttle = min(max(preds[0, 1].item(), 0), 1)
        print('\r', preds, end='')


def animate(i):
    bat_line.set_ydata(battery_history)
    force_line.set_ydata(force_history)
    return bat_line, force_line


if __name__ == '__main__':

    Thread(target=ser_thread).start()
    Thread(target=wheel_thread).start()
    Thread(target=camera_thread).start()
    Thread(target=inference_thread).start()

    a = animation.FuncAnimation(plt.gcf(), animate, blit=True, interval=100)
    plt.show()
    run = False
