import can
import time
import threading

ECU_QUERY = 0x7e0 # ecu
ECU_RESPONSE = ECU_QUERY + 8
ABS_QUERY = 0x760 # anti-lock brake system
ABS_RESPONSE = ABS_QUERY + 8
BC_QUERY = 0x726 # body control
BC_RESPONSE = BC_QUERY + 8
API_QUERY = 0x7d0 # accesory protocol interface
API_RESPONSE = API_QUERY + 8
SAS_QUERY = 0x797 # steering angle sensor
SAS_RESPONSE = API_QUERY + 8

msg_query_rpm = can.Message(arbitration_id = ECU_QUERY,
            data=[0x02, 0x01, 0x0c, 0x55, 0x55, 0x55, 0x55, 0x55,],
            extended_id=False)

msg_query_speed = can.Message(arbitration_id = ECU_QUERY,
            data=[0x03, 0x22, 0x15, 0x05, 0x55, 0x55, 0x55, 0x55,],
            extended_id=False)

msg_query_brake_pressure = can.Message(arbitration_id = ABS_QUERY,
            data=[0x03, 0x22, 0x20, 0x34, 0x55, 0x55, 0x55, 0x55,],
            extended_id=False)

msg_query_accelerator_fraction = can.Message(arbitration_id = ECU_QUERY,
            data=[0x03, 0x22, 0x03, 0x2b, 0x55, 0x55, 0x55, 0x55,],
            extended_id=False)

msg_query_steering_angle = can.Message(arbitration_id = ABS_QUERY,
              data=[0x03, 0x22, 0x33, 0x02, 0x55, 0x55, 0x55, 0x55,],
              extended_id=False)

msg_query_ignition_switch = can.Message(arbitration_id = BC_QUERY,
              data=[0x03, 0x22, 0x41, 0x1f, 0x55, 0x55, 0x55, 0x55,],
              extended_id=False)

msg_query_total_distance = can.Message(arbitration_id = ECU_QUERY,
              data=[0x03, 0x22, 0xdd, 0x01, 0x55, 0x55, 0x55, 0x55,],
              extended_id=False)

msg_query_gps = can.Message(arbitration_id = API_QUERY,
              data=[0x03, 0x22, 0x80, 0x12, 0x55, 0x55, 0x55, 0x55,],
              extended_id=False)

msg_query_gps_flow = can.Message(arbitration_id = API_QUERY,
            data=[0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,],
            extended_id=False)

class FordCAN(object):
    def __init__(self, channel = 'can0', bustype = 'socketcan_native'):
        self.bus = can.interface.Bus(channel=channel, bustype=bustype, can_filters = [
          {"can_id": ECU_RESPONSE, "can_mask": 0x7F, "extended": False},    
          {"can_id": ABS_RESPONSE, "can_mask": 0x7F, "extended": False},    
          {"can_id": BC_RESPONSE, "can_mask": 0x7F, "extended": False},    
          {"can_id": API_RESPONSE, "can_mask": 0x7F, "extended": False},    
        ])
        self.request_stop = False
        self.is_running = False

        self.frame_api = 0

        # callbacks to be overriden
        self.on_steering_wheel_angle = lambda x: 0
        self.on_accelerator_fraction = lambda x: 0
        self.on_brake_pressure = lambda x: 0
        self.on_rpm = lambda x: 0
        self.on_speed = lambda x: 0
        self.on_total_distance = lambda x: 0
        self.on_ignition_switch = lambda x: 0
        self.on_gps = lambda x: 0

    def start(self):
        self.request_stop = False

        self.thread_input = threading.Thread(target = self._input_loop)
        self.thread_input.daemon = True
        self.thread_input.start()
        self.thread_output = threading.Thread(target = self._output_loop)
        self.thread_output.daemon = True
        self.thread_output.start()
        self.thread_monitor = threading.Thread(target = self._monitor_loop)
        self.thread_monitor.daemon = True
        self.thread_monitor.start()

        self.is_running = True

    def stop(self):
        self.request_stop = True
        self.is_running = False

    def _output_loop(self):
        i = 0
        while not self.request_stop:
            time.sleep(0.005)
            i += 1
            try:
                self.bus.send(msg_query_steering_angle)
                time.sleep(0.005)
                if i % 4 == 0:
                    self.bus.send(msg_query_speed)
                    time.sleep(0.002)
                if i % 4 == 2:
                    self.bus.send(msg_query_brake_pressure)
                    time.sleep(0.002)
                if i % 8 == 1:
                    self.bus.send(msg_query_rpm)
                    time.sleep(0.002)
                if i % 8 == 3:
                    self.bus.send(msg_query_accelerator_fraction)
                    time.sleep(0.002)
                if i % 8 == 5:
                    self.bus.send(msg_query_ignition_switch)
                    time.sleep(0.002)
                if i % 8 == 7:
                    self.bus.send(msg_query_total_distance)
                    time.sleep(0.002)
                if i % 16 == 1:
                    time.sleep(0.002)
                    self.bus.send(msg_query_gps)
                    time.sleep(0.002)
                    self.bus.send(msg_query_gps_flow)

            except can.CanError:
                print("can error")
                continue

    def _input_loop(self):
        while not self.request_stop:
            time.sleep(0.001)
            try:
                message = self.bus.recv()
                if message.arbitration_id == ECU_RESPONSE:
                   self._process_ecu(message.data)
                elif message.arbitration_id == ABS_RESPONSE:
                   self._process_abs(message.data)
                elif message.arbitration_id == BC_RESPONSE:
                   self._process_bc(message.data)
                elif message.arbitration_id == API_RESPONSE:
                   self._process_api(message.data)
            except can.CanError:
                print("can error")
                continue

    def _monitor_loop(self):
        while not self.request_stop:
            time.sleep(0.5)
            if not self.thread_input.isAlive:
                self.stop()
            if not self.thread_output.isAlive:
                self.stop()
     
    def _process_ecu(self, data):
        if data[0:3] == b'\x04\x41\x0c':
            rpm = int.from_bytes(data[3:5], "big") / 4.0
            self.on_rpm(rpm)
        elif data[0:4] == b'\x05\x62\x15\x05':
            speed = int.from_bytes(data[4:6], "big") / 128.0
            self.on_speed(speed)
        elif data[0:4] == b'\x06\x62\xdd\x01':
            total_distance = int.from_bytes(data[4:7], "big") / 1.0
            self.on_total_distance(total_distance)
        elif data[0:4] == b'\x04\x62\x03\x2b':
            accelerator_fraction = data[4] / 255.0
            self.on_accelerator_fraction(accelerator_fraction)
        
    def _process_abs(self, data):
        if data[0:4] == b'\x05\x62\x33\x02':
            steering_wheel_angle = (int.from_bytes(data[4:6], "big") - 7800) / 10.0
            self.on_steering_wheel_angle(steering_wheel_angle)
        elif data[0:4] == b'\x05\x62\x20\x34':
            brake_pressure = int.from_bytes(data[4:6], "big", signed = True) * 30.0
            self.on_brake_pressure(brake_pressure)

    def _process_api(self, data):
        if data[0:5] == b'\x10\x12\x62\x80\x12':
            self.frame_api = 1
        if self.frame_api == 1 and data[0] == 0x21:
            lat = int.from_bytes(data[2:4], "big", signed=True) / 60.0
            lon = int.from_bytes(data[6:8], "big", signed=True) / 60.0
            self.on_gps((lat, lon))
        return

    def _process_bc(self, data):
        if data[0:4] == b'\x04\x62\x41\x1f':
            ignition_switch = data[4]
            self.on_ignition_switch(ignition_switch)

