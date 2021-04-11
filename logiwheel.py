from time import sleep
from pywinusb import hid
import struct
import math


UINT16_RANGE = (0, 65535)
INT16_RANGE = (-32768, 32767)
UINT10_RANGE = (0, 1024)
UINT8_RANGE = (0, 255)

pad_directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW', 'NONE']
alpha_buttons = ['A', 'B', 'X', 'Y']
aux_buttons = ['SIFT_UP', 'SHIFT_DOWN', 'MENU', 'WINDOW', 'RSB', 'LSB', 'XBOX', 'REVERSE']


def float_to_int16(val):
    return int(map_value(val, (-1, 1), INT16_RANGE))


def float_to_uint8(val):
    return int(map_value(val, (-1, 1), UINT8_RANGE))


def map_value(input, range_in, range_out):
    out = (input - range_in[0]) * (range_out[1] - range_out[0]) / (range_in[1] - range_in[0]) + range_out[0]
    if out > range_out[1]:
        out = range_out[1]
    if out < range_out[0]:
        out = range_out[0]
    return out


class LogiWheel:
    def __init__(self):
        self.VID = 0x046d
        self.PID = 0xc262
        devices = hid.HidDeviceFilter(vendor_id=0x046d, product_id=0xc262).get_devices()
        self.input_dvice = devices[0]
        self.out_short_device = devices[1]
        self.out_long_device = devices[2]
        self.on_input = lambda x: print("Callback not set", x)

    def __enter__(self):
        self.input_dvice.open()
        self.out_short_device.open()
        self.out_long_device.open()
        self.input_dvice.set_raw_data_handler(self.internal_callback)
        self.short_report = self.out_short_device.find_output_reports()[0]
        self.long_report = self.out_long_device.find_output_reports()[0]
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.input_dvice.close()
        self.out_short_device.close()
        self.out_long_device.close()

    def internal_callback(self, raw_data):
        data = struct.unpack('xBBBHBBBx', bytearray(raw_data))

        self.on_input({
            'steering': (2.0*data[-4] / UINT16_RANGE[1]) - 1,
            'throttle': 1 - data[-3] / 255,
            'brake': 1 - data[-2] / 255,
            'clutch': 1 - data[-1] / 255,
            'buttons': [x for i, x in enumerate(aux_buttons) if (1 << i) & data[1]] +
                       [x for i, x in enumerate(alpha_buttons) if (1 << i) & (data[0] >> 4)],
            'gear': int(math.log2(data[2])) + 1 if data[2] else 0,
            'pad_dir': pad_directions[data[0] & 0b00001111]
        })

    def set_degrees(self, degrees):
        packet = struct.pack('>4sH14x', b'\x11\xff\x0b\x6e', degrees)
        self.short_report.send(list(packet))

    def contant_effect(self, force, attack_level, attack_length, fade_level, fade_length, effect_id, auto_play=True):
        if auto_play:
            effect = 0x80
        else:
            effect = 0x00
        packet = struct.pack('>4sBBhBhBh50x', float_to_int16(effect_id), float_to_int16(effect),
                             float_to_int16(force), float_to_int16(attack_level), float_to_int16(attack_length),
                             float_to_int16(fade_level), float_to_int16(fade_length))
        self.long_report.send(list(packet))

    def spring_effect(self, center, left_coeff, right_coeff, left_sat=1, right_sat=1, dead_band=0,
                      effect_id=0, auto_play=True):
        if auto_play:
            effect = 0x86
        else:
            effect = 0x06
        packet = struct.pack('>4sBB4xhhhhhh42x', b'\x12\xff\x0b\x2e', effect_id, effect,
                             float_to_int16(left_sat), float_to_int16(left_coeff),
                             float_to_int16(dead_band), float_to_int16(center),
                             float_to_int16(right_coeff), float_to_int16(right_sat))
        self.long_report.send(list(packet))

    def damping_effect(self, center, left_coeff, right_coeff, left_sat=1, right_sat=1, dead_band=0,
                      effect_id=0, auto_play=True):
        if auto_play:
            effect = 0x87
        else:
            effect = 0x07
        packet = struct.pack('>4sBB4xhhhhhh42x', b'\x12\xff\x0b\x2e', effect_id, effect,
                             float_to_int16(left_coeff), float_to_int16(left_sat),
                             float_to_int16(dead_band), float_to_int16(center),
                             float_to_int16(right_coeff), float_to_int16(right_sat))
        self.long_report.send(list(packet))

    def stop_effect(self, effect_id=0):
        packet = struct.pack('4sBB14x', b'\x11\xff\x0b\x3e', effect_id, 0x01)
        self.short_report.send(list(packet))

    def play_effect(self, effect_id=0):
        packet = struct.pack('4sBB14x', b'\x11\xff\x0b\x3e', effect_id, 0x01)
        self.short_report.send(list(packet))

    def reset_effects(self):
        packet = struct.pack('4s16x', b'\x11\xff\x0b\x1e')
        self.short_report.send(list(packet))

    def set_force(self, percent):  # percent of 0xffff
        packet = struct.pack('4sH14x', b'\x11\xff\x0b\x8e', int(percent*65535))
        self.short_report.send(list(packet))
