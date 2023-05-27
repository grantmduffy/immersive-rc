from logiwheel import LogiWheel, map_value
from time import sleep, time, perf_counter
import serial
from serial.tools.list_ports import comports
import struct
import numpy as np
import cv2
from threading import Thread
from pywinusb import hid

display_res = (1280, 960)

# Select COM port
baudrate = 2000000
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
print(f'Connecting to {port}')

run = False
camera = 0

def camera_thread():
    global run
    cap = cv2.VideoCapture(camera)
    run = True
    while run:
        ret, frame = cap.read()
        if not ret:
            continue
        display_frame = cv2.resize(frame, display_res)
        cv2.imshow('FPV Camera', display_frame)
        cv2.waitKey(1)
        if cv2.getWindowProperty('FPV Camera', 0) != 0.0:
            run = False


inputs = {'steering': 0.0, 'throttle': 0.0, 'clutch': 0.0, 'brake': 0.0}
steering_pos = 0

def input_callback(x):
      global inputs
      inputs = x


def controller_callback(vals):
    global inputs
    inputs['steering'] = map_value((vals[2] << 8) + vals[1], (1, 65535), (-1, 1))
    throttle = (vals[4] << 8) + vals[3]
    inputs['throttle'] = map_value(throttle, (0, 65534), (0, 1)) if throttle < 65534 else 0
    # print(f'\r{(vals[2] << 8) + vals[1]:05d}|{(vals[4] << 8) + vals[3]:05d}', end='')
    # print(f'\r{inputs["steering"]:.2f}|{inputs["throttle"]:.2f}', end='')


if __name__ == '__main__':

    Thread(target=camera_thread).start()

    # for device in hid.find_all_hid_devices():
    #     print(device)

    # device = hid.HidDeviceFilter(vendor_id=0x1234, product_id=0xBEAD).get_devices()[0]
    # device = hid.HidDeviceFilter(vendor_id=0x045e, product_id=0x0000).get_devices()[0]
    device = hid.HidDeviceFilter(vendor_id=0x045e, product_id=0x028e).get_devices()[0]  # XBox 360 controller
    device.open()
    device.set_raw_data_handler(controller_callback)

    # while True:
    #     pass

    # with LogiWheel() as wheel, serial.Serial(port, timeout=0.1, baudrate=baudrate) as ser:
    with serial.Serial(port, timeout=0.1, baudrate=baudrate) as ser:

        # wheel.on_input = input_callback

        # read radio summary from arduino
        while ser.in_waiting == 0: pass
        sleep(1.0)
        print('FROM ARDUINO:')
        print(ser.read_all().decode('utf-8'))

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
            steering_pos = adc_vals[3]
            bat_val = map_value(adc_vals[4], (0, 2 ** 12), (0, 10.02), limit=False)

            t1 = perf_counter()

            print(f'\r{1 / (t1 - t0):7.2f}Hz', f'{steering_pos:5.3f}, steering={steering_pos:6.4f}, throttle={throttle_force:5.3}, bat={bat_val:.2f}V', end='    ')
            # sleep(1 / 60)
