import socket
import struct
import serial
from serial.tools import list_ports
import time

PORT = 65432 # Port to listen on (non-privileged ports are > 1023)


def get_arduino_serial_device():
    for device in list_ports.grep(".*"):
        if device.manufacturer == "FTDI":
            yield device.device


if __name__ == "__main__":
    '''Set up the structs to pack the data correctly for delivery'''
    # we pack and unpack the serial data as a 16 bit integer with the correct endian-ness
    serial_unpacker = struct.Struct('<h')
    serial_packer = struct.Struct('<h')

    '''Initialize the flags for read/write. Start with read first (send data to the client)'''
    reading = True
    writing = False

    '''Set up the serial port object'''
    ser_ports = list(get_arduino_serial_device())
    print("Found serial ports:")
    print(*ser_ports)
    if len(ser_ports) != 2:
      raise Exception(f"ERROR: {len(ser_ports)} Arduinos found. Quitting.") 

    # set your serial baudrate to 115200 on your haptkit
    ard_A = serial.Serial(ser_ports[0], 115200, timeout=10)
    ard_B = serial.Serial(ser_ports[1], 115200, timeout=10)

    old_time = time.time()  # just some time tracking for latency

    while True:
        current_time = time.time()
        old_time = current_time

        for (writer, reader, id) in [(ard_B, ard_A, 'A'), (ard_A, ard_B, 'B')]:
            recv = reader.read(2)  # read 2 bytes from the hapkit
            input = serial_unpacker.unpack(recv)
            print(f'{id} Pos: {input[0]/100000:0.3f}', end='\t')
            writer.write(recv)
            writer.flush()
        print(f'Latency: {(current_time - old_time)*1000:0.0f} ms')
