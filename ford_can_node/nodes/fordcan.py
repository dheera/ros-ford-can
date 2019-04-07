import can
import time
import threading

msg_query_rpm = can.Message(arbitration_id=0x7e0,
            data=[0x03, 0x22, 0xF4, 0x0C, 0x55, 0x55, 0x55, 0x55,],
            extended_id=False)

msg_query_speed = can.Message(arbitration_id=0x7e0,
            data=[0x03, 0x22, 0x15, 0x05, 0x55, 0x55, 0x55, 0x55,],
            extended_id=False)

msg_query_brake_pressure = can.Message(arbitration_id=0x760,
            data=[0x03, 0x22, 0x20, 0x34, 0x55, 0x55, 0x55, 0x55,],
            extended_id=False)

msg_query_steering_angle = can.Message(arbitration_id=0x760,
              data=[0x03, 0x22, 0x33, 0x02, 0x55, 0x55, 0x55, 0x55,],
              extended_id=False)

msg_query_ignition_switch = can.Message(arbitration_id=0x726,
              data=[0x03, 0x22, 0x41, 0x1f, 0x55, 0x55, 0x55, 0x55,],
              extended_id=False)


class FordCAN(object):
    def __init__(self, channel = 'can0', bustype = 'socketcan_native'):
        self.bus = can.interface.Bus(channel=channel, bustype=bustype)

    def start(self):
        self.thread_input = threading.Thread(target = self._input_loop)
        self.thread_input.daemon = True
        self.thread_input.start()
        self.thread_output = threading.Thread(target = self._output_loop)
        self.thread_output.daemon = True
        self.thread_output.start()

        self.steering_wheel_angle = 0.0, 0
        self.on_steering_wheel_angle = lambda x: 0
        self.speed = 0.0, 0
        self.on_speed = lambda x: 0
        self.brake_pressure = 0.0, 0
        self.on_brake_pressure = lambda x: 0
        self.rpm = 0.0, 0
        self.on_rpm = lambda x: 0
        self.ignition_switch = 0.0, 0
        self.on_ignition_switch = lambda x: 0

    def _output_loop(self):
        i = 0
        while True:
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
        while True:
            message = self.bus.recv()
            if message.arbitration_id == 0x7e8:
               self._process_obd(message.data)
            elif message.arbitration_id == 0x768:
               self._process_abs(message.data)
            elif message.arbitration_id == 0x72e:
               self._process_bc(message.data)
     
    def _process_obd(self, data):
        if data[0:4] == b'\x05\x62\xf4\x0c':
            self.rpm = int.from_bytes(data[4:6], "big") / 4.0, time.time()
            self.on_rpm(self.rpm[0])
        elif data[0:4] == b'\x05\x62\x15\x05':
            self.speed = int.from_bytes(data[4:6], "big") / 128.0, time.time()
            self.on_speed(self.speed[0])
        return
        
    def _process_abs(self, data):
        if data[0:4] == b'\x05\x62\x33\x02':
            self.steering_wheel_angle = (int.from_bytes(data[4:6], "big") - 7800) / 10.0, time.time()
            self.on_steering_wheel_angle(self.steering_wheel_angle[0])
        elif data[0:4] == b'\x05\x62\x20\x34':
            self.brake_pressure = int.from_bytes(data[4:6], "big", signed = True) * 30.0, time.time()
            self.on_brake_pressure(self.brake_pressure[0])

    def _process_bc(self, data):
        if data[0:4] == b'\x04\x62\x41\x1f':
            self.ignition_switch = data[4], time.time()
            self.on_ignition_switch(self.ignition_switch[0])

