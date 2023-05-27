from logiwheel import LogiWheel, map_value
from time import sleep, time, perf_counter
import serial
from serial.tools.list_ports import comports
import struct
import numpy as np
import cv2
from threading import Thread
from pywinusb import hid
from matplotlib import pyplot as plt

display_res = (1280, 960)

inputs = {'steering': 0.0, 'throttle': 0.0, 'clutch': 0.0, 'brake': 0.0}
steering_pos = 0
packet_rate = 500
frame_rate = 60
battery_level = 0

run = False
camera = 0
n_trace = 400
trace_scale = 2
trace_height = 100
trace_buffer = np.zeros((3, n_trace))
lines = np.zeros((3, n_trace, 2), int)
lines[..., 0] = np.arange(n_trace)[None, None, :] * trace_scale


def camera_thread():
    global run, frame_rate, packet_rate, battery_level, trace_buffer
    # cap = cv2.VideoCapture(camera)
    run = True
    while run:
        # ret, frame = cap.read()
        t0 = perf_counter()
        ret, frame = True, np.random.randint(0, 255, (480, 640, 3), np.uint8)
        if not ret:
            continue
        trace_buffer = np.roll(trace_buffer, 1, 1)
        trace_buffer[:, 0] = battery_level, frame_rate, packet_rate
        display_frame = cv2.resize(frame, display_res)

        # draw trace
        lines[..., 1] = (trace_height * trace_buffer / trace_buffer.max(axis=1, keepdims=True)).astype(int)
        for color, line in zip([(255, 0, 0), (0, 255, 0), (0, 0, 255)], lines):
            display_frame = cv2.polylines(display_frame, [line], False, color)

        # draw text
        for i, (t, c) in enumerate(zip(
                [f'{battery_level:7.2f}V', f'{frame_rate:7.2f}Hz', f'{packet_rate:7.2f}Hz'], 
                [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
            )):
            display_frame = cv2.putText(display_frame, t, (10, trace_height + 30 + i * 30), 
                                        cv2.FONT_HERSHEY_COMPLEX, 1, c, 2)

        cv2.imshow('FPV Camera', display_frame)
        cv2.waitKey(1)
        if cv2.getWindowProperty('FPV Camera', 0) != 0.0:
            run = False
        t1 = perf_counter()
        frame_rate = 1 / (t1 - t0)


def input_callback(x):
      global inputs
      inputs = x


def controller_callback(vals):
    global inputs
    inputs['steering'] = map_value((vals[2] << 8) + vals[1], (1, 65535), (-1, 1))
    throttle = (vals[4] << 8) + vals[3]
    inputs['throttle'] = map_value(throttle, (0, 65534), (0, 1)) if throttle < 65534 else 0


def get_port(baudrate=2000000):
    available_ports = comports()
    if len(available_ports) == 0:
        print('No COM ports available')
        exit(1)
    elif len(available_ports) == 1:
        port = available_ports[0].name
    else:
        print('Available COM ports:')
        print('\n'.join([x.name for x in available_ports]))
        port = input('Which port would you like to use?')
    return baudrate, port


if __name__ == '__main__':

    Thread(target=camera_thread).start()

    # device = hid.HidDeviceFilter(vendor_id=0x045e, product_id=0x0000).get_devices()[0]
    device = hid.HidDeviceFilter(vendor_id=0x045e, product_id=0x028e).get_devices()[0]  # XBox 360 controller
    device.open()
    device.set_raw_data_handler(controller_callback)

    # connect serial
    baud, port = get_port()
    print(f'Connecting to {port}')

    # with LogiWheel() as wheel, serial.Serial(port, timeout=0.1, baudrate=baudrate) as ser:
    with serial.Serial(port, timeout=0.1, baudrate=baudrate) as ser:

        # wheel.on_input = input_callback

        # read radio summary from arduino
        while ser.in_waiting == 0: pass
        sleep(1.0)
        print('FROM ARDUINO:')
        print(ser.read_all().decode('utf-8'))

        # wait for camera to start
        while not run:
            pass

        # start sending packets
        while run:
            if inputs is None:
                continue
            t0 = perf_counter()

            steering_input = map_value(inputs['steering'], (-0.5, 0.5), (-1, 1))
            # steering_force = map_value(err, (-0.5, 0.5), (-1, 1))
            throttle_force = inputs['throttle'] ** 0.5

            # encode packet
            packet = struct.pack(
                'HHHHH22s',
                int(round(map_value(inputs['brake'], (-1, 1), (2300, 20900)))) | (1 << 15),
                int(round(map_value(inputs['clutch'], (0, 1), (2300, 20900)))) | (1 << 15),
                int(round(map_value(throttle_force, (0, 1), (0, 1000)))),
                # int(round(map_value(abs(steering_force), (0, 1), (0, 1000)))) | (1 << 15),
                int(round(map_value(steering_input, (-1, 1), (500, 3600)))),
                0,
                b'Hello from Python!'
            )
            
            # send to car
            ser.write(packet)
            response = ser.read(32)
            if len(response) != 32:
                print('PACKET FAILED', len(response))
                continue
            
            echo = np.frombuffer(response[10:20], dtype=np.uint16)
            message = response[20:]
            if response[10:20] != packet[:10]:
                print('CORRUPTED PACKET', np.frombuffer(packet[:10], np.uint16), echo)
            if not response[20:].startswith(b'~running~'):
                print('NRF RESTART', response[20:])
            adc_vals = np.frombuffer(response[:10], dtype=np.int16)
            battery_level = map_value(adc_vals[4], (0, 2 ** 12), (0, 10.02), limit=False)

            t1 = perf_counter()
            packet_rate = 1 / (t1 - t0)

            # stop if battery gets too low
            if battery_level < 7.5:
                run = False

            # print(f'\r{1 / (t1 - t0):7.2f}Hz', f'{steering_pos:5.3f}, steering={steering_pos:6.4f}, throttle={throttle_force:5.3}, bat={bat_val:.2f}V', end='    ')
            # sleep(1 / 60)
