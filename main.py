from logiwheel import LogiWheel
from time import sleep
import struct
import serial
from threading import Thread
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np

ser_port = 'COM5'
baudrate = 115200
delay_time = 0.01

battery_scale = 412.63965
steering_offset = 0.0
steering_scale = 1.0

plot_n = 100
battery_history = np.zeros(plot_n)
force_history = np.zeros(plot_n)

plt.figure()
plt.subplot(211)
bat_line, = plt.plot(battery_history)
# plt.xlim(0, plot_n)
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


def update(data):
    global d, steering_offset, steering_scale
    d.update(data)
    if 'RSB' in d['buttons']:
        steering_offset = -d['steering']
    if 'LSB' in d['buttons']:
        steering_scale = 1 / abs(d['steering'])


def ser_thread():
    print('Serial thread starting')
    global d, run
    with serial.Serial(ser_port, baudrate) as ser:
        sleep(2)
        while run:
            ser.write(struct.pack('ff', -steering_scale * d['steering'] - steering_offset + 0.5, d['throttle']))
            d['force_feedback'], d['actual_steering_pos'], battery_level_raw, d['steering_force'] = struct.unpack('fHHf', ser.read(12))
            d['battery_level'] = battery_level_raw / battery_scale
            battery_history[:-1] = battery_history[1:]
            battery_history[-1] = d['battery_level']
            force_history[:-1] = force_history[1:]
            force_history[-1] = d['force_feedback']
            ser.flushInput()
            sleep(delay_time)


def animate(i):
    bat_line.set_ydata(battery_history)
    force_line.set_ydata(force_history)
    print(d['battery_level'], d['force_feedback'])
    return bat_line, force_line


with LogiWheel() as wheel:
    wheel.reset_all_effects()
    wheel.set_degrees(900)
    wheel.on_input = update
    effect_id = wheel.constant_effect(0)

    Thread(target=ser_thread).start()
    a = animation.FuncAnimation(plt.gcf(), animate, blit=True, interval=100)
    plt.show()
    run = False
