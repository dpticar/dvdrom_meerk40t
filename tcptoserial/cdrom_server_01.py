#!/usr/bin/env python3

import socket
import serial
from time import sleep

ser = serial.Serial(port='COM5', baudrate=500000, timeout=.1)

alcrc = 0

#for crc debug
def printAC():
    ser.write(b"\xAF")
    ser.flush()
    acrc = ser.read()
    print(acrc.hex())
    print(hex(alcrc))


def convert_to_list_bytes(data):
    if isinstance(data, str):  # python 2
        return [ord(e) for e in data]
    else:
        return [e for e in data]


def wait_until_accepting_packets():
    print("WAITING uap")
    ser.write(b"\xA0")
    ser.flush()
    while True:
        x = ser.read()
        if x == b"\xCE":
            #print("w finished")

            return
        if x == b"\xEE":
            ser.write(b"\xA0")
            ser.flush()
            #print("busy")



def onewire_crc_lookup(line):

    global alcrc
    """
    License: 2-clause "simplified" BSD license
    Copyright (C) 1992-2017 Arjen Lentz
    https://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table

    :param line: line to be CRC'd
    :return: 8 bit crc of line.
    """
    crc = 0

    for i in range(0, 30):
        crc = line[i] ^ crc
        alcrc = line[i] ^ alcrc
        crc = crc_table[crc & 0x0F] ^ crc_table[16 + ((crc >> 4) & 0x0F)]
        alcrc = crc_table[alcrc & 0x0F] ^ crc_table[16 + ((alcrc >> 4) & 0x0F)]
    return crc


crc_table = [
    0x00,
    0x5E,
    0xBC,
    0xE2,
    0x61,
    0x3F,
    0xDD,
    0x83,
    0xC2,
    0x9C,
    0x7E,
    0x20,
    0xA3,
    0xFD,
    0x1F,
    0x41,
    0x00,
    0x9D,
    0x23,
    0xBE,
    0x46,
    0xDB,
    0x65,
    0xF8,
    0x8C,
    0x11,
    0xAF,
    0x32,
    0xCA,
    0x57,
    0xE9,
    0x74,
]


# Connect to the server with `telnet $HOSTNAME 5000`.

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setblocking(False)
server.bind(('127.0.0.1', 1022))
server.listen(1)

connection = None
buffer = b''
accepting_packets = False
while True:
    if connection is None:
        try:
            connection, address = server.accept()
        except BlockingIOError:
            pass
    else:
        try:
            message = connection.recv(1024)
        except BlockingIOError:
            pass
        else:
            buffer = buffer + message
            
    if not buffer:
        continue
    

    find = buffer.find(b"\n", 0, 30)
    if find == -1:  # No end found.
        length = min(30, len(buffer))
    else:  # Line end found.
        length = min(30, len(buffer), find + 1)
    packet = bytes(buffer[:length])

    if packet.endswith(b"\n"):
        packet = packet[:-1]

    if len(packet) != 0:
        if packet.endswith(b"#"):
            packet = packet[:-1]
            try:
                c = packet[-1]
            except IndexError:
                c = b"F"  # Packet was simply #. We can do nothing.
            packet += bytes([c]) * (30 - len(packet))  # Padding. '\n'
        else:
            packet += b"F" * (30 - len(packet))  # Padding. '\n'

    if len(packet) == 30:
        packet = b"\xA6\x00" + packet + b"\xA6" + \
            bytes([onewire_crc_lookup(packet)])

    if(accepting_packets == False):
            wait_until_accepting_packets()
    ser.write(packet)
    ser.flush()
    print("writing packet :", packet, " buffer : ", len(buffer))
    ser.write(b"\xA0")
    ser.flush()
    for attempts in range(300):
        x = ser.read()
        if x == b"\xCE":
            print("OK1 ",hex(alcrc))
            buffer = buffer[length:]
            accepting_packets = True
            break
        if x == b"\xEE":
            print("BUSY ")
            buffer = buffer[length:]
            accepting_packets = False
            break
        if x == b"\xCF":
            print("CRC error ")
            accepting_packets = True
            break
