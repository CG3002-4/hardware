#include "LowPower.h"
#include "Wire.h"


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h
//files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyroIC1(0x68);
MPU6050 accelgyroIC2(0x69);

#define AD0_PIN_0 4  // Connect this pin to the AD0 pin on IMU #0
#define AD0_PIN_1 5  // Connect this pin to the AD0 pin on IMU #1
#define LED_PIN 13

int16_t ax1, ay1, az1;
int16_t gx1, gy1, gz1;

int16_t ax2, ay2, az2;
int16_t gx2, gy2, gz2;

int analogPin0 = 0;     // potentiometer wiper (middle terminal) connected to analog pin 0
                       // outside leads to ground and +5V
int analogPin1 = 1;     // potentiometer wiper (middle terminal) connected to analog pin 1
                       // outside leads to ground and +5V
int val0 = 0;           // variable to store the value read
int val1 = 0;           // variable to store the value read
float voltage1 =0;
float voltage2 =0;
float current1=0;
float current2=0;
float power1=0;
float power2=0;
float energy1=0;
float energy2=0;

float currenttime;
float pasttime;

bool blinkState = false;

void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
Wire.begin();

// initialize serial communication
// (38400 chosen because it works as well at 8MHz as it does at 16MHz,
//but
// it's really up to you depending on your project)
Serial.begin(38400);

// initialize device
Serial.println("Initializing I2C devices...");
//accelgyro.initialize();
accelgyroIC1.initialize();
accelgyroIC2.initialize();

// verify connection
Serial.println("Testing device connections...");
Serial.println(accelgyroIC1.testConnection() ? "MPU6050 #1 connection successful" : "MPU6050 connection failed");
Serial.println(accelgyroIC2.testConnection() ? "MPU6050 #2 connection successful" : "MPU6050 connection failed");

// configure Arduino LED for
pinMode(13, OUTPUT);
pinMode(A0,INPUT);
pinMode(A1,INPUT);

}

void loop() {

  pasttime=millis();

  //digitalWrite(13,HIGH);
  //delay(500);
  //digitalWrite(13,LOW);
  //delay(500);
  //LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  
// read raw accel/gyro measurements from device
accelgyroIC1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
accelgyroIC2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

// these methods (and a few others) are also available
//accelgyro.getAcceleration(&ax, &ay, &az);
//accelgyro.getRotation(&gx, &gy, &gz);
  currenttime=millis();

// display tab-separated accel/gyro x/y/z values
Serial.print("MPU1:\t");
Serial.print(ax1/16384.0, 2); Serial.print("\t");
Serial.print(ay1/16384.0, 2); Serial.print("\t");
Serial.print(az1/16384.0, 2); Serial.print("\t");
Serial.print(gx1/131, 2); Serial.print("\t");
Serial.print(gy1/131, 2); Serial.print("\t");
Serial.println(gz1/131, 2);


// display tab-separated accel/gyro x/y/z values
Serial.print("MPU2:\t");
Serial.print(ax2/16384.0, 2); Serial.print("\t");
Serial.print(ay2/16384.0, 2); Serial.print("\t");
Serial.print(az2/16384.0, 2); Serial.print("\t");
Serial.print(gx2/131, 2); Serial.print("\t");
Serial.print(gy2/131, 2); Serial.print("\t");
Serial.println(gz2/131, 2);


  val0 = analogRead(A0);     // read the input pin0 for voltage divider
  voltage1=(val0*5.0/1023.0*2);
  current1=(voltage1/(2.0*75000.0));
  power1=(voltage1*current1); 
  energy1=(power1*(currenttime-pasttime));
  
  Serial.print(F("VD : "));
  Serial.print(voltage1, 2);  Serial.println("V");
  Serial.print(current1*1000.0, 2);Serial.println("mA");
  Serial.print(power1*1000.0, 2);Serial.println("mW");
  Serial.println(energy1, 2);Serial.println("J");
 
  
  val1 = analogRead(A1);     // read the input pin1 for current sensor 
  voltage2=(val1*5.0/1023.0);
  current2=((voltage2*1000.0)/(0.1*10000.0));
  power2=(voltage2*current2); 
  energy2=(power2*(currenttime-pasttime));
  Serial.print(F("CSV: "));
  Serial.print(voltage2, 2);  Serial.println("V");
  Serial.print(current2*1000.0, 2);Serial.println("mA");
  Serial.print(power2*1000.0, 2);Serial.println("mW");
  Serial.print(energy2, 2);Serial.println("J");
  Serial.println();

delay(500);

// blink LED to indicate activity
//blinkState = !blinkState;
//digitalWrite(LED_PIN, blinkState);
}
