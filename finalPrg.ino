/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  This example runs directly on ESP32 chip.

  Note: This requires ESP32 support package:
    https://github.com/espressif/arduino-esp32

  Please be sure to select the right ESP32 module
  in the Tools -> Board menu!

  Change WiFi ssid, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

/* Fill-in your Template ID (only if using Blynk.Cloud) */
//#define BLYNK_TEMPLATE_ID   "YourTemplateID"

#include <ESP32Servo.h>
#include <Wire.h> 
#include <SPI.h> 
#include <RTClib.h>
#include "Melody.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#define SERIAL_BAUDRATE 115200
#define PIN_TONE 12

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "vWPVAZn4ExabzAEG0NF8g-VGFZoKCwmD";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "linksys";
char pass[] = "8023338737";

const int PushButton=15; 
Melody frereJacques("(cdec)x2   (efgr)x2   ((gagf)-ec)x2     (c g_ c+)x2");
RTC_DS1307 RTC;     // Setup an instance of DS1307 naming it RTC
Servo myservo;
Servo myservo1;
int pos = 0;    // variable to store the servo position
int servoPin = 18;
int servoPin1 = 14;
int n;
int foundState;


void setup()
{
  // Debug console
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);
  pinMode(PushButton, INPUT);
   #ifdef ESP_PLATFORM
      #define CHANNEL 5
      ledcSetup(CHANNEL, 5000, 8);
      ledcAttachPin(PIN_TONE, CHANNEL);
      ledcWrite(CHANNEL, 0); //duty Cycle de 0
  #endif
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50); 
  myservo1.setPeriodHertz(50);// standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000);
  myservo1.attach(servoPin1, 1000, 2000);// attaches the servo on pin 18 to the servo object  
}

BLYNK_WRITE(V0)
{
motors(); 
}


void loop()
{
  Blynk.run();
  Serial.println("scan start");
  deviceStatus();
  music();

}

void deviceStatus(){
    // WiFi.scanNetworks will return the number of networks found
    n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        foundState = 0;
        for (int i = 0; i < n; ++i) {
            // Print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            if (String("test").equals(WiFi.SSID(i))) {

             foundState = 1;
             Serial.println("DEVICE  FOUND! !!!!");
             Serial.println(foundState);
              }
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(")");
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
            delay(10);
        }
        if (foundState == 0){
          Serial.println("DEVICE NOT FOUND! !!!!");
               Blynk.notify("PET not in range");
          }
    }
    Serial.println("");

    // Wait a bit before scanning again
    delay(500);   
}


void play(Melody melody) {
  Serial.print("Melody length : ");
  Serial.println(melody.length()); //Get the total length (number of notes) of the melody.
  melody.restart(); //The melody iterator is restarted at the beginning.

  //While there is a next note to play.
  while (melody.hasNext()) {
        melody.next(); //Move the melody note iterator to the next one.
        unsigned int frequency = melody.getFrequency(); //Get the frequency in Hz of the curent note.
        unsigned long duration = melody.getDuration();  //Get the duration in ms of the curent note.
        unsigned int loudness = melody.getLoudness();   //Get the loudness of the curent note (in a subjective relative scale from -3 to +3).
                                                        //Common interpretation will be -3 is really soft (ppp), and 3 really loud (fff).
        if (frequency > 0){
            tone(PIN_TONE, frequency);
            setLoudness(loudness);
        }
        else{
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

void music(){         
  // digitalRead function stores the Push button state 
  // in variable push_button_state
  int Push_button_state = digitalRead(PushButton);
  Serial.println(Push_button_state);
  // if condition checks if push button is pressed
  // if pressed LED will turn on otherwise remain off 
  if ( Push_button_state == HIGH ){ 
    play(frereJacques);
  }
}

void customtimer()
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
  //FOR ESP Platform, pin is unused
  void tone(int pin, int frequency) {
      ledcWriteTone(CHANNEL, frequency);
  }
  //FOR ESP Platform, pin is unused
  void noToneCustom(int pin) 
  {
    ledcWrite(CHANNEL, 0);
  }
#endif

void motors()
{
music();
    for (pos = 0; pos <= 120; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);    // tell servo to go to position in variable 'pos'
    delay(15);             // waits 15ms for the servo to reach the position
  }
  music();
  for (pos = 120; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);    // tell servo to go to position in variable 'pos'
    delay(15);             // waits 15ms for the servo to reach the position
  }
  //   music();
  //   for (pos = 0; pos <= 360; pos += 1)
  //   { // goes from 0 degrees to 180 degrees
  //       // in steps of 1 degree
  //       myservo.write(pos);    // tell servo to go to position in variable 'pos'
  //       delay(5);             // waits 15ms for the servo to reach the position
  //   }
  //   music();
  //   for (pos = 360; pos >= 0; pos -= 1) 
  //   { // goes from 180 degrees to 0 degrees
  //       myservo.write(pos);    // tell servo to go to position in variable 'pos'
  //       delay(15);             // waits 15ms for the servo to reach the position
  //   }
  //  music();
  // for (pos = 0; pos <= 360; pos += 1)
  // { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo1.write(pos);    // tell servo to go to position in variable 'pos'
  //   delay(5);             // waits 15ms for the servo to reach the position
  // }
  // music();
  // for (pos = 360; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
  //   myservo1.write(pos);    // tell servo to go to position in variable 'pos'
  //   delay(15);             // waits 15ms for the servo to reach the position
  // }
  // music();
}

