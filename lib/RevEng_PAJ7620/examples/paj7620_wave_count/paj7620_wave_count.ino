/*
  Example Script: paj7620_wave_count.ino
  Package: RevEng_PAJ7620

  Copyright (c) 2020 Aaron S. Crandall
  Author       : Aaron S. Crandall <crandall@gonzaga.edu>
  Modified Time: December 2020

  Description: This demo uses the getWaveCount() interface.
    As you wave your hand in front of the sensor, it counts the quantity
    of passes you make, and the RevEng library exposes that value.
    This demo outputs that value and the wave event.

  License: Same as package under MIT License (MIT)
*/

// Includes enum definition of GES_* return values from readGesture()
#include "RevEng_PAJ7620.h"

// Create gesture sensor driver object
RevEng_PAJ7620 sensor = RevEng_PAJ7620();

// Last seen number of waves pulled from sensor
int curr_wave_count = 0;


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
  Serial.println("Wave your hand to get a wave count (max 15).");
}


// ******************************************************************
void loop()
{
  int new_wave_count = 0;
  new_wave_count = sensor.getWaveCount();     // Query for new wave count

  if( new_wave_count != curr_wave_count ) {   // Only print if a new value happens
    curr_wave_count = new_wave_count;
    Serial.print("New wave count: ");
    Serial.println(curr_wave_count);
  }

  // Gesture is an enumerated type defined in RevEng_PAJ7620.h
  // That's where the GES_UP, GES_DOWN, etc values come from
  Gesture gesture;

  // Read gesture from sensor - returns a value of GES_* from GESTURES enum in RevEng_PAJ7620.h
  gesture = sensor.readGesture();

  if (gesture == GES_WAVE)
  {
    Serial.println("GES_WAVE event");
  }

  delay(100);
}