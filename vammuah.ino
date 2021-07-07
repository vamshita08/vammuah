#include "Melody.h"
#include <ESP32Servo.h>

#include <Wire.h> // Library for I2C communication
#include <SPI.h>  // not used here, but needed to prevent a RTClib compile error
#include <RTClib.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define SERIAL_BAUDRATE 115200
#define PIN_TONE 12
// We assigned a name LED pin to pin number 22

// this will assign the name PushButton to pin numer 15
const int PushButton=15; 
Melody frereJacques("(cdec)x2   (efgr)x2   ((gagf)-ec)x2     (c g_ c+)x2");
RTC_DS1307 RTC;     // Setup an instance of DS1307 naming it RTC
Servo myservo;
Servo myservo1;


int scanTime = 5; //In seconds
BLEScan* pBLEScan;
int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin = 18;
int servoPin1 = 14;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    }
};

// This Setup function is used to initialize everything 
void setup()
{
// This statement will declare pin 15 as digital input 
  pinMode(PushButton, INPUT);
  Serial.begin(115200);
    #ifdef ESP_PLATFORM
        #define CHANNEL 5
        ledcSetup(CHANNEL, 5000, 8);
        ledcAttachPin(PIN_TONE, CHANNEL);
        ledcWrite(CHANNEL, 0); //duty Cycle de 0
    #endif
Serial.println("Scanning...");
BLEDevice::init("");
pBLEScan = BLEDevice::getScan(); //create new scan
pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
pBLEScan->setInterval(100);
pBLEScan->setWindow(99);  // less or equal setInterval value
Wire.begin(); // Start the I2C
RTC.begin();  // Init RTC
RTC.adjust(DateTime(__DATE__, __TIME__));  // Time and date is expanded to date and time on your computer at compiletime
Serial.print('Time and date set');
// Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myservo.setPeriodHertz(50); 
     myservo1.setPeriodHertz(50);// standard 50 hz servo
    myservo.attach(servoPin, 1000, 2000);
    myservo1.attach(servoPin1, 1000, 2000);// attaches the servo on pin 18 to the servo object
    // using default min/max of 1000us and 2000us
    // different servos may require different min/max settings
    // for an accurate 0 to 180 sweep


}

void loop()
{ 
  scan();
  timer();
  motors();
  //input from button and play users audio
}

void play(Melody melody) {
  Serial.print("Melody length : ");
  Serial.println(melody.length()); //Get the total length (number of notes) of the melody.
  melody.restart(); //The melody iterator is restarted at the beginning.

  while (melody.hasNext()) //While there is a next note to play.
  {
        melody.next(); //Move the melody note iterator to the next one.
        unsigned int frequency = melody.getFrequency(); //Get the frequency in Hz of the curent note.
        unsigned long duration = melody.getDuration();  //Get the duration in ms of the curent note.
        unsigned int loudness = melody.getLoudness();   //Get the loudness of the curent note (in a subjective relative scale from -3 to +3).
                                                        //Common interpretation will be -3 is really soft (ppp), and 3 really loud (fff).
        if (frequency > 0)
        {
            tone(PIN_TONE, frequency);
            setLoudness(loudness);
        }
        else
        {
            noToneCustom(PIN_TONE);
        }

        delay(duration);

        //This 1 ms delay with no tone is added to let a "breathing" time between each note.
        //Without it, identical consecutives notes will sound like just one long note.
       noToneCustom(PIN_TONE);
        delay(1);
    }

    noToneCustom(PIN_TONE);
    delay(1000);
}


void setLoudness(int loudness){
    //Loudness could be use with a mapping function, according to your buzzer or sound-producing hardware
   #ifdef ESP_PLATFORM
      #define MIN_HARDWARE_LOUDNESS 0
      #define MAX_HARDWARE_LOUDNESS 16
      ledcWrite(CHANNEL, map(loudness, -4, 4, MIN_HARDWARE_LOUDNESS, MAX_HARDWARE_LOUDNESS));
   #endif

}
void music()
{
          // digitalRead function stores the Push button state 
        // in variable push_button_state
        int Push_button_state = digitalRead(PushButton);
        Serial.println(Push_button_state);
        // if condition checks if push button is pressed
        // if pressed LED will turn on otherwise remain off 
        if ( Push_button_state == HIGH )
        { 
        play(frereJacques);
        }
}
void scan()
{
  music();
  BLEScanResults foundDevices = pBLEScan->start(1, false);
  Serial.print("Devices found: ");
  Serial.println(foundDevices.getCount());
  Serial.println("Scan done!");
  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
  delay(2000);
}
void timer()
{
   DateTime now = RTC.now();
 
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

}


#ifdef ESP_PLATFORM

    void tone(int pin, int frequency) //FOR ESP Platform, pin is unused
    {
        ledcWriteTone(CHANNEL, frequency);
    }
    void noToneCustom(int pin) //FOR ESP Platform, pin is unused
  {
      ledcWrite(CHANNEL, 0);
    }

#endif

void motors()
{
    music();
    for (pos = 0; pos <= 360; pos += 1)
    { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        myservo.write(pos);    // tell servo to go to position in variable 'pos'
        delay(5);             // waits 15ms for the servo to reach the position
    }
    music();
    for (pos = 360; pos >= 0; pos -= 1) 
    { // goes from 180 degrees to 0 degrees
        myservo.write(pos);    // tell servo to go to position in variable 'pos'
        delay(15);             // waits 15ms for the servo to reach the position
    }
   music();
  for (pos = 0; pos <= 360; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo1.write(pos);    // tell servo to go to position in variable 'pos'
    delay(5);             // waits 15ms for the servo to reach the position
  }
  music();
  for (pos = 360; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo1.write(pos);    // tell servo to go to position in variable 'pos'
    delay(15);             // waits 15ms for the servo to reach the position
  }
  music();

}
