#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Raspberry Pi to Arduino I2C Communication
#i2cdetect -y 1

#library
import sys
import smbus2 as smbus
import time

# Slave Addresses
I2C_SLAVE_ADDRESS = 11 # address for slave number 1, BREATH
I2C_SLAVE2_ADDRESS = 12 # address for slave number 2
I2C_SLAVE3_ADDRESS = 13 # address for slave number 3

# This function converts a string to an array of bytes.
def ConvertStringsToBytes(src):
  converted = []
  for b in src:
    converted.append(ord(b))
  return converted

def main():
    # Create the I2C bus
    I2Cbus = smbus.SMBus(1)
    with smbus.SMBus(1) as I2Cbus:
        # slaveSelect = input("Which Firebeetle (1-3): ")
        slaveSelect = "1"
        # cmd = input("Enter command: ")
        cmd = "go"

        if slaveSelect == "1":
            slaveAddress = I2C_SLAVE_ADDRESS
            print("Slave 1 selected for breath signals")
        elif slaveSelect == "2":
            slaveAddress = I2C_SLAVE2_ADDRESS
            print("Slave 2 selected")
        elif slaveSelect == "3":
            slaveAddress = I2C_SLAVE3_ADDRESS
            print("Slave 3 selected")
        else:
            print("Wrong slave selected")
            quit()
        BytesToSend = ConvertStringsToBytes(cmd)
        print("Sent to slave address: " + str(slaveAddress) + " the command: " + str(cmd))
        print(str(len(BytesToSend)) + " Bytes sent: ")
        print(BytesToSend)

        I2Cbus.write_i2c_block_data(slaveAddress, 0x00, BytesToSend)
        time.sleep(1)
        I2Cbus.read_i2c_block_data(slaveAddress,0x00,16)
        I2Cbus.write_i2c_block_data(slaveAddress, 0x00, BytesToSend)
        time.sleep(1)

        while True:
            try:
                data2 = I2Cbus.read_i2c_block_data(slaveAddress,0x00,len(BytesToSend)+1)
                print(str(len(data2)-1) + " Bytes recieved from slave: ")
                print(data2[1:])
                break
            except:
                print("remote i/o error")
                time.sleep(0.5)
    return 0

if __name__ == '__main__':
    main()
