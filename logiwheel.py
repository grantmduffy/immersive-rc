from time import sleep
from pywinusb import hid
import struct


UINT16_RANGE = (0, 65535)
INT16_RANGE = (-32768, 32767)
UINT10_RANGE = (0, 1024)
UINT8_RANGE = (0, 255)


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
        data = struct.unpack('bbbbHbbbx', bytearray(raw_data))
        steering_angle = (2.0*data[4] / 65535) - 1
        self.on_input(steering_angle)

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

    def spring_effect(self, left_coeff, left_sat, dead_band, center, right_coeff, right_sat, effect_id, auto_play=True):
        if auto_play:
            effect = 0x86
        else:
            effect = 0x06
        packet = struct.pack('>4sBB4xhhhhhh42x', b'\x12\xff\x0b\x2e', effect_id, effect,
                             float_to_int16(left_coeff), float_to_int16(left_sat),
                             float_to_int16(dead_band), float_to_int16(center),
                             float_to_int16(right_coeff), float_to_int16(right_sat))
        self.long_report.send(list(packet))

    def damping_effect(self, left_coeff, left_sat, dead_band, center, right_coeff, right_sat, effect_id, auto_play=True):
        if auto_play:
            effect = 0x87
        else:
            effect = 0x07
        packet = struct.pack('>4sBB4xhhhhhh42x', b'\x12\xff\x0b\x2e', effect_id, effect,
                             float_to_int16(left_coeff), float_to_int16(left_sat),
                             float_to_int16(dead_band), float_to_int16(center),
                             float_to_int16(right_coeff), float_to_int16(right_sat))
        self.long_report.send(list(packet))

    def stop_effect(self, effect_id):
        packet = struct.pack('4sBB14x', b'\x11\xff\x0b\x3e', effect_id, 0x01)
        self.short_report.send(list(packet))

    def play_effect(self, effect_id):
        packet = struct.pack('4sBB14x', b'\x11\xff\x0b\x3e', effect_id, 0x01)
        self.short_report.send(list(packet))

    def reset_effects(self):
        packet = struct.pack('4s16x', b'\x11\xff\x0b\x1e')
        self.short_report.send(list(packet))

    def set_force(self, percent):  # percent of 0xffff
        packet = struct.pack('4sH14x', b'\x11\xff\x0b\x8e', int(percent*65535))
        self.short_report.send(list(packet))
