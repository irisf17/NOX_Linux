#include <Wire.h>
#include <Arduino.h>
#include <stdlib.h>

#define I2C_SLAVE_ADDRESS 11

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

// pwm 0-255

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

void loop()
{
  if (flag_breath == 1)
  {
    breath_signals(7384);
  }
}
