import glob

import numpy as np
import serial


class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

        # self.sensor_data = []
        # self.sensor_list = []
        # try:
        #     self.sensor_data_ser = serial.Serial(glob.glob('/dev/ttyUSB*')[0], 115200)
        #     print("/dev/ttyUSB* connected succeed")
        # except:
        #     self.sensor_data_ser = None
        # self.sensor_data_max_len = 51

        # try:
        #     self.lidar_ser = serial.Serial(glob.glob('/dev/ttyACM*')[0], 230400, timeout=1)
        #     print("/dev/ttyACM* connected succeed")
        # except:
        #     self.lidar_ser = None

        self.ANGLE_PER_FRAME = 12
        self.HEADER = 0x54
        self.lidar_angles = []
        self.lidar_distances = []
        self.lidar_angles_show = []
        self.lidar_distances_show = []
        self.last_start_angle = 0
        self.breath_light_flag = True

    def readline(self):
        sentinel = b"\r\n"
        i = self.buf.find(sentinel)
        if i >= 0:
            r = self.buf[:i + len(sentinel)]
            self.buf = self.buf[i + len(sentinel):]
            return r
        while True:
            i = max(1, min(512, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(sentinel)
            if i >= 0:
                r = self.buf + data[:i + len(sentinel)]
                self.buf[0:] = data[i + len(sentinel):]
                return r
            else:
                self.buf.extend(data)

    def clear_buffer(self):
        self.s.reset_input_buffer()

    def read_sensor_data(self):
        if self.sensor_data_ser == None:
            return

        try:
            buffer_clear = False
            while self.sensor_data_ser.in_waiting > 0:
                buffer_clear = True
                sensor_readline = self.sensor_data_ser.readline()
                if len(sensor_readline) <= self.sensor_data_max_len:
                    self.sensor_list.append(
                        sensor_readline.decode('utf-8')[:-2])
                else:
                    self.sensor_list.append(sensor_readline.decode(
                        'utf-8')[:self.sensor_data_max_len])
                    self.sensor_list.append(sensor_readline.decode(
                        'utf-8')[self.sensor_data_max_len:-2])
            if buffer_clear:
                self.sensor_data = self.sensor_list.copy()
                self.sensor_list.clear()
                self.sensor_data_ser.reset_input_buffer()
        except Exception as e:
            print(f"[base_ctrl.read_sensor_data] error: {e}")

    def parse_lidar_frame(self, data):
        # header = data[0]
        # verlen = data[1]
        # speed  = data[3] << 8 | data[2]
        start_angle = (data[5] << 8 | data[4]) * 0.01
        # print(start)
        # end_angle = (data[43] << 8 | data[42]) * 0.01
        for i in range(0, self.ANGLE_PER_FRAME):
            offset = 6 + i * 3
            distance = data[offset + 1] << 8 | data[offset]
            confidence = data[offset + 2]
            # lidar_angles.append(np.radians(start_angle + i * 0.167))
            self.lidar_angles.append(np.radians(
                start_angle + i * 0.83333 + 180))
            # lidar_angles.append(np.radians(start_angle + end_angle))
            self.lidar_distances.append(distance)
        # end_angle = (data[43] << 8 | data[42]) * 0.01
        # timestamp = data[45] << 8 | data[44]
        # crc = data[46]
        return start_angle

    def lidar_data_recv(self):
        if self.lidar_ser == None:
            return
        try:
            while True:
                self.header = self.lidar_ser.read(1)
                if self.header == b'\x54':
                    # Read the rest of the data
                    data = self.header + self.lidar_ser.read(46)
                    hex_data = [int(hex(byte), 16) for byte in data]
                    start_angle = self.parse_lidar_frame(hex_data)
                    if self.last_start_angle > start_angle:
                        break
                    self.last_start_angle = start_angle
                else:
                    self.lidar_ser.flushInput()

            self.last_start_angle = start_angle
            self.lidar_angles_show = self.lidar_angles.copy()
            self.lidar_distances_show = self.lidar_distances.copy()
            self.lidar_angles.clear()
            self.lidar_distances.clear()
        except Exception as e:
            print(f"[base_ctrl.lidar_data_recv] error: {e}")
            self.lidar_ser = serial.Serial(
                glob.glob('/dev/ttyACM*')[0], 230400, timeout=1)
