import can
import time
import threading

ECU_QUERY = 0x7e0 # ecu
ECU_RESPONSE = OBD_QUERY + 8
ABS_QUERY = 0x760 # anti-lock brake system
ABS_RESPONSE = ABS_QUERY + 8
BC_QUERY = 0x760 # body control
BC_RESPONSE = BC_QUERY + 8


msg_query_rpm = can.Message(arbitration_id = ECU_QUERY,
            data=[0x03, 0x22, 0xF4, 0x0C, 0x55, 0x55, 0x55, 0x55,],
            extended_id=False)

msg_query_speed = can.Message(arbitration_id = ECU_QUERY,
            data=[0x03, 0x22, 0x15, 0x05, 0x55, 0x55, 0x55, 0x55,],
            extended_id=False)

msg_query_brake_pressure = can.Message(arbitration_id = ABS_QUERY,
            data=[0x03, 0x22, 0x20, 0x34, 0x55, 0x55, 0x55, 0x55,],
            extended_id=False)

msg_query_steering_angle = can.Message(arbitration_id = ABS_QUERY,
              data=[0x03, 0x22, 0x33, 0x02, 0x55, 0x55, 0x55, 0x55,],
              extended_id=False)

msg_query_ignition_switch = can.Message(arbitration_id = BC_QUERY,
              data=[0x03, 0x22, 0x41, 0x1f, 0x55, 0x55, 0x55, 0x55,],
              extended_id=False)

class FordCAN(object):
    def __init__(self, channel = 'can0', bustype = 'socketcan_native'):
        self.bus = can.interface.Bus(channel=channel, bustype=bustype)
        self.request_stop = False

    def start(self):
        self.request_stop = False

        self.thread_input = threading.Thread(target = self._input_loop)
        self.thread_input.daemon = True
        self.thread_input.start()
        self.thread_output = threading.Thread(target = self._output_loop)
        self.thread_output.daemon = True
        self.thread_output.start()

        # callbacks to be overriden
        self.on_steering_wheel_angle = lambda x: 0
        self.on_speed = lambda x: 0
        self.on_brake_pressure = lambda x: 0
        self.on_rpm = lambda x: 0
        self.on_ignition_switch = lambda x: 0

    def stop(self):
        self.request_stop = True

    def _output_loop(self):
        i = 0
        while not self.request_stop:
            time.sleep(0.01)
            i += 1
            try:
                self.bus.send(msg_query_steering_angle)
                time.sleep(0.001)
                if i % 3 == 0:
                    self.bus.send(msg_query_rpm)
                    time.sleep(0.001)
                if i % 3 == 1:
                    self.bus.send(msg_query_speed)
                    time.sleep(0.001)
                if i % 3 == 2:
                    self.bus.send(msg_query_brake_pressure)
                if i % 20 == 0:
                    self.bus.send(msg_query_ignition_switch)
            except can.CanError:
                print("can error")
                continue

    def _input_loop(self):
        while not self.request_stop:
            try:
                message = self.bus.recv()
                if message.arbitration_id == ECU_RESPONSE:
                   self._process_ecu(message.data)
                elif message.arbitration_id == ABS_RESPONSE:
                   self._process_abs(message.data)
                elif message.arbitration_id == BC_RESPONSE:
                   self._process_bc(message.data)
            except can.CanError:
                print("can error")
                continue
     
    def _process_ecu(self, data):
        if data[0:4] == b'\x05\x62\xf4\x0c':
            rpm = int.from_bytes(data[4:6], "big") / 4.0
            self.on_rpm(rpm)
        elif data[0:4] == b'\x05\x62\x15\x05':
            speed = int.from_bytes(data[4:6], "big") / 128.0
            self.on_speed(speed)
        
    def _process_abs(self, data):
        if data[0:4] == b'\x05\x62\x33\x02':
            steering_wheel_angle = (int.from_bytes(data[4:6], "big") - 7800) / 10.0
            self.on_steering_wheel_angle(steering_wheel_angle)
        elif data[0:4] == b'\x05\x62\x20\x34':
            brake_pressure = int.from_bytes(data[4:6], "big", signed = True) * 30.0
            self.on_brake_pressure(brake_pressure)

    def _process_bc(self, data):
        if data[0:4] == b'\x04\x62\x41\x1f':
            ignition_switch = data[4]
            self.on_ignition_switch(ignition_switch)

