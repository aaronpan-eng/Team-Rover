# SPDX-FileCopyrightText: 2018 Brent Rubell for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
Example for using the RFM9x Radio with Raspberry Pi.

Learn Guide: https://learn.adafruit.com/lora-and-lorawan-for-raspberry-pi
Author: Brent Rubell for Adafruit Industries
"""
# Import Python System Libraries
import time
import sys
from digitalio import DigitalInOut, Direction, Pull

# Import Blinka Libraries
import busio
import board

# Import RFM9x
import adafruit_rfm9x

# Updates for 2/16 test
import subprocess
import platform

# Updates for UART from LoRA
import threading

def UART_timestamping():
    subprocess.run(["python3", "/home/username/uart_timestamp_read_in.py"])

def UART_DAC_PING():
    subprocess.run(["python3", "/home/username/DAC_PING.py"])

def timestamping_status():
    if timestamping.is_alive():
        return "TIMESTAMPING ALIVE"
    elif not timestamping.is_alive():
        return "TIMESTAMPING DEAD"

timestamping = threading.Thread(target=UART_timestamping)

# Create the I2C interface.
i2c = busio.I2C(board.SCL, board.SDA)

# Configure LoRa Radio
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
rfm9x.tx_power = 23

this_hostname = platform.node()

while True:
    # Get user input for the message to send
#    message_to_send = input("Enter message to send: ")

    # Wait for a packet to be received
    packet = rfm9x.receive(timeout=10)
    if packet is not None:
        # Decode and print the received message
        print(packet)
        try:
            received_message = packet.decode("utf-8")
        except:
            print("null string error")
        # print("")
        print(received_message)
        if this_hostname in received_message:
            if "echo" in received_message:
                rfm9x.send(bytes(received_message, "utf-8"))
                print("Message sent:", received_message)
            if "transmit" in received_message:
                UART_DAC_PING()
                rfm9x.send(bytes("DAC PING SENT", "utf-8"))
            if "timestamp" in received_message:
                f = open("/home/username/timestamp.txt", "r")
                timestamp = f.read()
                f.close()
                rfm9x.send(bytes(timestamp," utf-8"))
                f = open("/home/username/timestamp.txt", "w")
                f.write("invalid")
                f.close()
            if "uart" in received_message:
                str = timestamping_status()
                rfm9x.send(bytes(str," utf-8"))
        if "all" in received_message:
            if "uart" in received_message:
                if not timestamping.is_alive():
                    timestamping = threading.Thread(target=UART_timestamping)
                    timestamping.start()
                str = timestamping_status()
                rfm9x.send(bytes(str," utf-8"))