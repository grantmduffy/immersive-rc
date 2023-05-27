# Reference:
# https://github.com/jackun/logitech-ff/blob/master/hid-logitech-hidpp/hid-logitech-hidpp.c
# https://opensource.logitech.com/opensource/images/c/ce/Logitech_Force_Feedback_Protocol_V1.5.pdf
# file:///C:/Users/duffyg/Downloads/logitech_hidpp_2.0_specification_draft_2012-06-04.pdf


from pywinusb import hid
import struct
import math
from time import time, sleep


UINT16_RANGE = (0, 65535)
INT16_RANGE = (-32768, 32767)
UINT10_RANGE = (0, 1024)
UINT8_RANGE = (0, 255)

pad_directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW', 'NONE']
alpha_buttons = ['A', 'B', 'X', 'Y']
aux_buttons = ['SHIFT_UP', 'SHIFT_DOWN', 'MENU', 'WINDOW', 'RSB', 'LSB', 'XBOX', 'REVERSE']


def float_to_int16(val):
    return int(map_value(val, (-1, 1), INT16_RANGE))


def float_to_uint16(val):
    return int(map_value(val, (0, 1), UINT16_RANGE))


def float_to_uint8(val):
    return int(map_value(val, (-1, 1), UINT8_RANGE))


def map_value(input, range_in, range_out, limit=True):
    out = (input - range_in[0]) * (range_out[1] - range_out[0]) / (range_in[1] - range_in[0]) + range_out[0]
    if limit:
        out = min(out, range_out[1])
        out = max(out, range_out[0])
    return out


class LogiWheel:

    def __init__(self):
        self.VID = 0x046d
        self.PID = 0xc262
        devices = hid.HidDeviceFilter(vendor_id=0x046d, product_id=0xc262).get_devices()
        self.input_device = devices[0]
        self.out_short_device = devices[1]
        self.out_long_device = devices[2]
        self.on_input = lambda x: print("Callback not set", x)
        self.short_response = None
        self.long_response = None
        self.response_timout = 3.0

    def __enter__(self):
        self.input_device.open()
        self.out_short_device.open()
        self.out_long_device.open()
        self.input_device.set_raw_data_handler(self.internal_callback)
        self.short_report = self.out_short_device.find_output_reports()[0]
        self.long_report = self.out_long_device.find_output_reports()[0]
        self.out_short_device.set_raw_data_handler(self.short_response_callback)
        self.out_long_device.set_raw_data_handler(self.long_response_callback)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.input_device.close()
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

    def short_response_callback(self, data):
        self.short_response = data

    def long_response_callback(self, data):
        self.long_response = data

    def get_short_response(self):
        t_start = time()
        while time() - t_start > self.response_timout:
            if self.short_response is not None:
                data = self.short_response
                self.short_response = None
                return data
        raise TimeoutError(f'Failed to get response in {self.response_timout}s')

    def get_long_response(self):
        t_start = time()
        while time() - t_start < self.response_timout:
            if self.long_response is not None:
                data = self.long_response
                self.long_response = None
                return data
        raise TimeoutError(f'Failed to get response in {self.response_timout}s')

    def get_info(self):
        response = self.send_packet(b'\x11\xff\x0b\x0e')
        slot_count, actuator_mask = response[4], response[5]
        return slot_count, actuator_mask

    def constant_effect(self, force, effect_id=0, play=True):
        effect_type = 0x00
        if play:
            effect_type |= 0x80
        packet = struct.pack('>4sBB4xh8x', b'\x11\xff\x0b\x2e', effect_id, effect_type, float_to_int16(force))
        response = self.send_packet(packet)
        return response[4]

    # TODO: get working
    def spring_effect(self, center, left_coeff, right_coeff, left_sat=1, right_sat=1, dead_band=0,
                      effect_id=0, auto_play=True):
        if auto_play:
            effect = 0x86
        else:
            effect = 0x06
        packet = struct.pack('>4sBB4xHhhhhH', b'\x12\xff\x0b\x2e', effect_id, effect,
                             float_to_uint16(left_sat), float_to_int16(left_coeff),
                             float_to_int16(dead_band), float_to_int16(center),
                             float_to_int16(right_coeff), float_to_uint16(right_sat))
        response = self.send_packet(packet)
        return response[4]

    def reset_all_effects(self):
        packet = b'\x11\xff\x0b\x1e'
        self.send_packet(packet)

    def set_effect_state(self, effect_id, state=0):
        packet = struct.pack('4sBB', b'\x11\xff\x0b\x3e', effect_id, state)
        response = self.send_packet(packet)
        return response[5]

    # OLD STUFF:
    def set_degrees(self, degrees):
        packet = struct.pack('>4sH14x', b'\x11\xff\x0b\x6e', degrees)
        self.send_packet(packet)

    # TODO: Get working
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

    def send_packet(self, data):
        self.long_response = None
        if data[0] == 0x11:
            data += b'\x00' * (20 - len(data))
            self.short_report.send(data)
        else:
            data += b'\x00' * (64 - len(data))
            self.long_report.send(data)
        return self.get_long_response()


if __name__ == '__main__':
    with LogiWheel() as wheel:
        wheel.on_input = lambda x: None
        wheel.reset_all_effects()
        sleep(1)
        id = wheel.constant_effect(1)
        sleep(0.5)
        wheel.constant_effect(-1, id)
        sleep(1)
        wheel.constant_effect(0, id)
        print(wheel.set_effect_state(id))


