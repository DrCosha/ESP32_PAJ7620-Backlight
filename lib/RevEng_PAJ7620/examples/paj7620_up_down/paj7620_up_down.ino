/*
  Example Script: paj7620_up_down.ino
  Package: RevEng_PAJ7620

  Copyright (c) 2020 Aaron S. Crandall
  Author       : Aaron S. Crandall <crandall@gonzaga.edu>
  Modified Time: December 2020

  Description: This demo only uses the up and down gestures to control the builtin LED
    Moving your hand in front of the sensor up will turn the LED on
    Moving down will turn the LED off

  License: Same as package under MIT License (MIT)
*/

// Includes enum definition of GES_* return values from readGesture()
#include "RevEng_PAJ7620.h"

// Create gesture sensor driver object
RevEng_PAJ7620 sensor = RevEng_PAJ7620();


// ******************************************************************
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);     // Configure LED for output

  Serial.begin(115200);

  if( !sensor.begin() )             // return value of 1 == success
  {
    Serial.print("PAJ7620 init error -- device not found -- halting");
    while(true) {}
  }

  Serial.println("PAJ7620U2 init: OK.");
  Serial.println("Wave your hand up to turn on light, and down to turn it off.");
}


// ******************************************************************
void loop()
{
  // Gesture is an enumerated type defined in RevEng_PAJ7620.h
  // That's where the GES_UP, GES_DOWN, etc values come from
  Gesture gesture;

  // Read gesture from sensor - returns a value of GES_* from GESTURES enum in RevEng_PAJ7620.h
  gesture = sensor.readGesture();

  if (gesture == GES_UP)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("GES_UP -- Light on");
  }
  else if(gesture == GES_DOWN)
  {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("GES_DOWN -- Light off");
  }

  delay(100);
}