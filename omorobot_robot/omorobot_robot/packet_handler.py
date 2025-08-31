import time
import serial
from rclpy.logging import get_logger

class PacketHandler:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.timeout = 0.1
        try:
            self.ser.open()
        except serial.SerialException as e:
            raise RuntimeError(f'Failed to open serial port {port}: {e}')
        if self.ser.is_open:
            self.print(f'serial port {self.ser.name} is opened')
        else:
            raise RuntimeError('Serial port open error')
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.incoming_data = ['ENC', 'GYRO', 'ODO', 'POSE', 'VW']
        self._bat = [0.0, 0.0, 0.0]
        self._enc = [0, 0]
        self._gyro = [0.0, 0.0, 0.0]
        self._odo = [0.0, 0.0]
        self._pose = [0.0, 0.0, 0.0]
        self._vw = [0.0, 0.0]
        self.print('Serial init complete')

    def print(self, str_info):
        get_logger('packet_handler').info(str_info)

    def try_reconnect(self):
        if not self.ser.readable():
            return
        else:
            self.ser.close()
        while not self.ser.is_open:
            try:
                self.ser.open()
                self.print(f'reconnect to {self.ser.port}')
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except serial.SerialException:
                self.print('reconnect failed.')
                time.sleep(1)
    
    def write_data(self, tx_string):
        if not self.ser.is_open:
            raise RuntimeError('Serial port is not open.')
        try:
            self.ser.write((tx_string + '\r\n').encode())
        except serial.SerialException as e:
            self.print(f'Serial write error: {e}')

    def close_port(self):
        try:
            self.write_data('$cPEEN,0')
        except Exception as e:
            self.print('Error while disabling periodic info: {e}')
        finally:
            if self.ser.is_open:
                self.ser.close()
                self.print('Serial port close')

    def get_battery_state(self):
        self.write_data('$qBAT')

    def odo_reset(self):
        self.write_data('$cODO,0')

    def vw_command(self, lin_vel, ang_vel):
        self.write_data('$cVW,{:.0f},{:.0f}'.format(lin_vel, ang_vel))

    def set_periodic_info(self, millisecond):
        for idx, item in enumerate(self.incoming_data):
            self.write_data('$cREGI,' + str(idx) + ',' + item)
            self.print(self.ser.readline().strip())
            time.sleep(0.001)
        self.write_data('$cPERI,' + str(millisecond))
        self.print(self.ser.readline().strip())
        self.write_data('$cPEEN,1')
        self.print(self.ser.readline().strip())
        self.print('set periodic info complete')

    def read_packet(self):
        try:
            whole_packet = (self.ser.readline().split(b'\r')[0]).decode('utf-8').strip()
            if not whole_packet or whole_packet[0] != '#':
                return
            # self.print(whole_packet)                                                    # for debugging
            packet = whole_packet.split(',')
            header = packet[0].split('#')[1]
            if header.startswith('ENC'):                                                # encoder
                self._enc = [int(packet[1]), int(packet[2])]
            elif header.startswith('GYRO'):                                             # gyro
                self._gyro = [float(packet[1]), float(packet[2]), float(packet[3])]
            elif header.startswith('ODO'):                                              # omdometer(mm)
                self._odo = [float(packet[1]), float(packet[2])]
            elif header.startswith('POSE'):                                             # roll, pitch, yaw
                self._pose = [float(packet[1]), float(packet[2]), float(packet[3])]
            elif header.startswith('VW'):                                               # lin_vel, ang_vel
                self._vw = [float(packet[1]), float(packet[2])]
            elif header.startswith('BAT'):
                self._bat = [float(packet[1]), float(packet[2]), float(packet[3])]
        except Exception as e:
            self.print(f'Serial read error: {e}')
            self.try_reconnect()
