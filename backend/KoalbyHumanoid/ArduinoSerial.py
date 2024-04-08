import time
import serial
from serial.tools import list_ports

class ArduinoSerial(object):

    def __init__(self):
        # to run connected to Arduino
        ports = list_ports.comports() # Auto detects COM port
        self.ser = serial.Serial(ports[0].device, 115200, timeout=1)

        self.ser.reset_input_buffer()
        time.sleep(3)

    def send_command(self, command):  # sends a command to the arduino from the RasPi
        message = str.encode(command + "\n") # Firmware looks for '\n' as command terminator
        self.ser.write(message)

    def read_float(self): # reads a float message from the arduino (all messages are floats)
        line = self.ser.readline()
        line = line.decode('utf-8').strip()
        return line
