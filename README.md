# Master Slave setup for Dummy Patient SNORTUR

## Table of contents
- General info
- Setup
- Connections
- Features
- Inspiration
- Explaining the code



## General info

Using I2C to implement master slave communication between devices to simplify the system for patient dummy Snortur. RaspberryPi is the master and Firebeetle ESP32 is the slave. Python language is used on the RaspberryPi, C++ is used for the Firebeetle. 


## Setup

Many slaves can be connected to one master. The slaves are assigned with different 8-bit addresses to define between them. 

Platformio(extension in VScode) is used to program the Firebeetle.

SSH interface (extension in VScode) is used to program the RaspberryPi.

Install smbus2 on RaspberryPi to be able to run the code.
```
>> pip3 install smbus2
```


Libraries used on Firebeetle ESP32, (are included at the top of the script.)



```
Wire.h

Arduino.h

stdlin.h 
```



## Connections

The devices are connected through 2 GPIO pins the transmit and receive pins. 

•	Connect the SDA (I2C data) of the Pi (pin 2) to the Firebeetle SDA.

•	Connect the SCL (I2C clock) of the Pi (pin 3) to the Firebeetle SCL.

You should really pay attention when you connect 2 pins between those boards. Usually you’d have to use a level converter between 3.3V and 5V. But in this specific case we can avoid using one. (because the Firebeetle is a slave then it does not matter)
RaspberryPi runs og 3.3 V, Firebeetle on 5V.


Figure showing the connection between RaspberryPi and Arduino. Is similar to connecting between RaspberryPi and Firebeetle ESP32.  

![Connections between the RaspberryPi and Firebeetle ESP32](wires.png)






## Inspiration

Setup is inspired from this website.

https://www.aranacorp.com/en/communication-between-raspberry-pi-and-arduino-with-i2c/ 



## Explaining the code

__Raspberry Pi__ - using python

Importing libraries

```python
#library
import sys
import smbus2 as smbus
import time
```
Choosing addresses to acknowlegde between multiple slaves. In our setup we only use one for the Breath system. The slave address 11 is the one for the Breath system.
```python
# Slave Addresses
I2C_SLAVE_ADDRESS = 11 # address for slave number 1, BREATH
I2C_SLAVE2_ADDRESS = 12 # address for slave number 2
I2C_SLAVE3_ADDRESS = 13 # address for slave number 3
```
Function that is used to convert string to bytes to be able to send it over to the slave. The I2C communication transmits and recieves bytes only.
```python
# This function converts a string to an array of bytes.
def ConvertStringsToBytes(src):
  converted = []
  for b in src:
    converted.append(ord(b))
  return converted
```

main function, used to transmit and recieve data between master and slave. cmd is the string that is sent to the slave. The string needs to contain the letter g to activate the breath code in the slave/Firebeetle. Note only one g, not two, three or more. An example of a string is "go", "green", "g". Is also case sensitive. So only include small lettered g.
The string can not exceed the size of 14 characters. Because the maximum buffer is 15 in the Firebeetle. (an extra byte is transmitted from the master to the slave.)
```python
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
```

Run code
```python
if __name__ == '__main__':
    main()

```

__Firebeetle ESP32__ - using C++

Include libraries
```c++
#include <Wire.h>
#include <Arduino.h>
#include <stdlib.h>
```

Define the slave address
```c++
#define I2C_SLAVE_ADDRESS 11
```

Define global variables
```c++
// // GLOBAL variable
// Define a buffer for receiving data
char data_to_echo = 0;
int buffer[15] = {};
int counter = 1;
char flag_breath = 0;

// Initializing pins
const int motorPin1 = D2;
const int motorPin1_hi = D4;
const int motorPin1_lo = D5;

```
Breath function to control the motor pumps using PWM. This will need to be modified to improve the pressure signals.
```c++
void breath_signals(int value)
{
  for (int i = 0; i < value; i++)
  {
    digitalWrite(motorPin1_hi, HIGH);
    digitalWrite(motorPin1_lo, LOW);
    analogWrite(motorPin1, 200); // breath out --- turn on the motor2, using pwm
    delay(500);
    analogWrite(motorPin1, 210); // turn on the motor2, using pwm
    delay(200);
    analogWrite(motorPin1, 220); // turn on the motor2, using pwm
    delay(100);
    analogWrite(motorPin1, 250); // turn on the motor2, using pwm
    delay(500);
    digitalWrite(motorPin1_hi, LOW);
    analogWrite(motorPin1, 0); // turn on the motor2, using pwm
    delay(200);
    digitalWrite(motorPin1_hi, HIGH);
    analogWrite(motorPin1, 200); // turn on the motor2, using pwm
    delay(400);
    digitalWrite(motorPin1_hi, LOW);
    analogWrite(motorPin1, 0); // turn on the motor2, using pwm
    delay(2000);
  }
}
```
Send data to master using the wire library.
```c++
void sendData()
{
  Serial.println("----> SENDING");
  int array_len = sizeof(buffer) / sizeof(int);
  flag_breath = 0;
  for (char j = 0; j < array_len; j++)
  {
    Wire.write(buffer[j]);
    Serial.print("sending buffer: ");
    Serial.println(buffer[j]);
    if (buffer[j] == 103 && counter % 2 == 0)
    {
      Serial.println("EURIKA!");
      // test code for blinking led
      // blinkLED();
      // PRESSURE FUNCTION HERE!
      flag_breath = 1;
      // breath_signals(5);
    }
  }
  counter++;
}

```

Receive data from master using the wire library.
```c++
void receiveData(int numBytes)
{
  Serial.print(numBytes);
  Serial.println(" bytes recieved");
  Serial.println(F("----> recieved events"));
  for (int i = 0; i < numBytes; i++)
  {
    data_to_echo = Wire.read();
    Serial.print("recieved value : ");
    Serial.println(data_to_echo);
    buffer[i] = data_to_echo;
  }
  Serial.println("BUFFER that is read");
  for (char m = 0; m < numBytes; m++)
  {
    Serial.print(buffer[m]);
    Serial.print(" ");
  }
  Serial.println();
}
```

Initialize every in setup.
```c++
void setup() // init everything!
{
  Wire.begin(I2C_SLAVE_ADDRESS);
  // initialize breath system
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin1_hi, OUTPUT);
  pinMode(motorPin1_lo, OUTPUT);

  Serial.begin(9600);
  delay(100);
  Serial.println(F("----------I am Slave1----------"));
  delay(1000);
  Wire.onRequest(sendData);
  Wire.onReceive(receiveData);
}
```


Main loop, does nothing. The receive and transmit is activated through interrupt.
```c++
void loop()
{
  if (flag_breath == 1)
  {
    breath_signals(7384);
  }
}

```