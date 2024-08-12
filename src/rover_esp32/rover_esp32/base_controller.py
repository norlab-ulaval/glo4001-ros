import json
import queue
import threading
import time
from pathlib import Path
import serial
import yaml
from ament_index_python.packages import get_package_share_directory
import struct

from .read_line import ReadLine

pkg_path = Path(get_package_share_directory('rover_esp32'))
with open(pkg_path / 'config/config.yaml', 'r') as yaml_file:
    config = yaml.safe_load(yaml_file)


class BaseController:
    def __init__(self, uart_dev_set, baud_set):
        self.ser = serial.Serial(uart_dev_set, baud_set, timeout=1)
        self.ser.flush()
        self.rl = ReadLine(self.ser)
        self.command_queue = queue.Queue()
        self.command_thread = threading.Thread(
            target=self.process_commands, daemon=True)
        self.command_thread.start()

        self.base_light_status = 0
        self.head_light_status = 0

        self.data_buffer = None
        self.base_data = None

        self.use_lidar = config['base_config']['use_lidar']
        self.extra_sensor = config['base_config']['extra_sensor']

    def read_feedback(self):
        try:
            while self.rl.s.in_waiting > 0:
                self.data_buffer = self.rl.readline()
                if len(self.data_buffer) > 30:
                    self.base_data = self.data_buffer
                    self.data_buffer = None
                    return self._parse_bytes_array(self.base_data)
            self.rl.clear_buffer()
            self.data_buffer = self.rl.readline()
            self.base_data = self.data_buffer
            return self._parse_bytes_array(self.base_data)
        except Exception as e:
            self.rl.clear_buffer()

    def _parse_bytes_array(self, data):
        # Contains the values for
        # FEEDBACK_BASE_INFO, speedGetA, speedGetB, gx, gy, gz, ax, ay, az, mx, my, mz, rgx, rgy, rgz, rax, ray, raz, rmx, rmy, rmz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset, en_odom_l, en_odom_r, loadVoltage_V
        # All of the values are doubles (8 bytes)
        keys = ['FEEDBACK_BASE_INFO', 'speedGetA', 'speedGetB', 'gx', 'gy', 'gz', 'ax', 'ay', 'az', 'mx', 'my', 'mz', 'rgx', 'rgy', 'rgz', 'rax', 'ray',
                'raz', 'rmx', 'rmy', 'rmz', 'ax_offset', 'ay_offset', 'az_offset', 'gx_offset', 'gy_offset', 'gz_offset', 'en_odom_l', 'en_odom_r', 'loadVoltage_V']
        values = struct.unpack('f' * 31, data)
        return dict(zip(keys, values))

    def read_feedback_json(self):
        try:
            while self.rl.s.in_waiting > 0:
                self.data_buffer = json.loads(
                    self.rl.readline().decode('utf-8'))
                if 'T' in self.data_buffer:
                    self.base_data = self.data_buffer
                    self.data_buffer = None
                    if self.base_data["T"] == 1003:
                        return self.base_data
            self.rl.clear_buffer()
            self.data_buffer = json.loads(self.rl.readline().decode('utf-8'))
            self.base_data = self.data_buffer
            return self.base_data
        except Exception as e:
            self.rl.clear_buffer()
            # print(f"[base_ctrl.feedback_data] error: {e}\nraw data: {self.rl.readline().decode('utf-8')}")

    def on_data_received(self):
        self.ser.reset_input_buffer()
        data_read = json.loads(self.rl.readline().decode('utf-8'))
        return data_read

    def send_command(self, data):
        self.command_queue.put(data)

    def process_commands(self):
        while True:
            data = self.command_queue.get()
            self.ser.write((json.dumps(data) + '\n').encode("utf-8"))

    # Commands from https://github.com/norlab-ulaval/glo4001-esp32-robot/blob/main/ROS_Driver/json_cmd.h
    def set_feedback_flow(self, enabled):
        cmd = 1 if enabled else 0
        data = {"T": 131, "cmd": cmd}
        self.send_command(data)

    def set_feedback_delay_ms(self, delay):
        data = {"T": 142, "cmd": delay}
        self.send_command(data)

    def get_feedback(self):
        data = {"T": 130}
        self.send_command(data)

    # def gimbal_emergency_stop(self):
    #     data = {"T": 0}
    #     self.send_command(data)

    # def base_speed_ctrl(self, input_left, input_right):
    #     data = {"T": 1, "L": input_left, "R": input_right}
    #     self.send_command(data)

    # def gimbal_ctrl(self, input_x, input_y, input_speed, input_acceleration):
    #     data = {"T": 133, "X": input_x, "Y": input_y,
    #             "SPD": input_speed, "ACC": input_acceleration}
    #     self.send_command(data)

    # def gimbal_base_ctrl(self, input_x, input_y, input_speed):
    #     data = {"T": 141, "X": input_x, "Y": input_y, "SPD": input_speed}
    #     self.send_command(data)

    # def base_oled(self, input_line, input_text):
    #     data = {"T": 3, "lineNum": input_line, "Text": input_text}
    #     self.send_command(data)

    # def base_default_oled(self):
    #     data = {"T": -3}
    #     self.send_command(data)

    # def bus_servo_id_set(self, old_id, new_id):
    #     # data = {"T":54,"old":old_id,"new":new_id}
    #     data = {"T": config['cmd_config']
    #             ['cmd_set_servo_id'], "raw": old_id, "new": new_id}
    #     self.send_command(data)

    # def bus_servo_torque_lock(self, input_id, input_status):
    #     # data = {"T":55,"id":input_id,"status":input_status}
    #     data = {"T": config['cmd_config']['cmd_servo_torque'],
    #             "id": input_id, "cmd": input_status}
    #     self.send_command(data)

    # def bus_servo_mid_set(self, input_id):
    #     # data = {"T":58,"id":input_id}
    #     data = {"T": config['cmd_config']['cmd_set_servo_mid'], "id": input_id}
    #     self.send_command(data)

    # def lights_ctrl(self, pwmA, pwmB):
    #     data = {"T": 132, "IO4": pwmA, "IO5": pwmB}
    #     self.send_command(data)
    #     self.base_light_status = pwmA
    #     self.head_light_status = pwmB

    # def base_lights_ctrl(self):
    #     if self.base_light_status != 0:
    #         self.base_light_status = 0
    #     else:
    #         self.base_light_status = 255
    #     self.lights_ctrl(self.base_light_status, self.head_light_status)

    # def gimbal_dev_close(self):
    #     self.ser.close()

    # def change_breath_light_flag(self, input_cmd):
    #     self.breath_light_flag = input_cmd

    # def breath_light(self, input_time):
    #     self.change_breath_light_flag(True)
    #     breath_start_time = time.monotonic()
    #     while time.monotonic() - breath_start_time < input_time:
    #         for i in range(0, 128, 10):
    #             if not self.breath_light_flag:
    #                 self.lights_ctrl(0, 0)
    #                 return
    #             self.lights_ctrl(i, 128 - i)
    #             time.sleep(0.1)
    #         for i in range(0, 128, 10):
    #             if not self.breath_light_flag:
    #                 self.lights_ctrl(0, 0)
    #                 return
    #             self.lights_ctrl(128 - i, i)
    #             time.sleep(0.1)
    #     self.lights_ctrl(0, 0)


class BaseControllerSingleton(BaseController):
    inst = None

    def __new__(cls, *args, **kwargs):
        if not cls.inst:
            cls.inst = super().__new__(cls)
        return cls.inst
