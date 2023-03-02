/*
  Example Script: paj7620_interrupt.ino
  Package: RevEng_PAJ7620

  Copyright (c) 2017 Marc Finns
  Author       : MarcFinns
  Modified Time: March 2017
  
  Modified:
  2020 - Aaron S. Crandall <crandall@gonzaga.edu>

  Description: This demo can recognize 9 gestures and output the result,
    including move up, move down, move left, move right, move forward,
    move backward, circle-clockwise, circle-counter clockwise, and wave.
    This example uses an interrupt handler to detect when a gesture has
    occurred instead of polling for it.

  Special wiring: sensor INT pin to microcontroller (Arduino) pin 2

  License: Same as package under MIT License (MIT)
*/

// Includes enum definition of GES_* return values from readGesture()
#include "RevEng_PAJ7620.h"

#define INTERRUPT_PIN 2                     // Interrupt capable Arduino pin 

// Note: When using interrupts, any variables used both in and out of the
//  interrupt routine should be declared volatile.
volatile bool isr = false;                  // isr == Interrupt Service Routine

RevEng_PAJ7620 sensor = RevEng_PAJ7620();   // Create gesture sensor API/Object

// ***************************************************************************
void setup()
{
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.begin(115200);
  Serial.println("PAJ7620 Test Demo: Recognize 9 gestures using interrupt callback.");

  if( !sensor.begin() )                     // Returns 0 if sensor connect fail
  {
    Serial.print("PAJ7620 I2C error - halting");
    while(true) {}
  }

  Serial.println("PAJ7620 Init OK.");
  Serial.println("Please input your gestures:");

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interruptRoutine, FALLING);
}


// ***************************************************************************
void loop()
{

  Gesture gesture;                  // Gesture is an enum type from RevEng_PAJ7620.h
  if (isr == true)                  // See interruptRoutine for where this is set to true
  {
    isr = false;                    // Reset ISR flag for next interrupt
    gesture = sensor.readGesture(); // Read back current gesture (if any) of type Gesture

    switch (gesture)
    {
      case GES_FORWARD:
        {
          Serial.print(" GES_FORWARD");
          break;
        }

      case GES_BACKWARD:
        {
          Serial.print(" GES_BACKWARD");
          break;
        }

      case GES_LEFT:
        {
          Serial.print(" GES_LEFT");
          break;
        }

      case GES_RIGHT:
        {
          Serial.print(" GES_RIGHT");
          break;
        }

      case GES_UP:
        {
          Serial.print(" GES_UP");
          break;
        }

      case GES_DOWN:
        {
          Serial.print(" GES_DOWN");
          break;
        }

      case GES_CLOCKWISE:
        {
          Serial.print(" GES_CLOCKWISE");
          break;
        }

      case GES_ANTICLOCKWISE:
        {
          Serial.print(" GES_ANTICLOCKWISE");
          break;
        }

      case GES_WAVE:
        {
          Serial.print(" GES_WAVE");
          break;
        }

      case GES_NONE:
        {
          Serial.print(" GES_NONE");
          break;
        }
    }

    Serial.print(", Gesture Code: ");
    Serial.println(gesture);

    // If a second gesture happens while the first is being handled,
    // isr might be set true again already, to be handled on the next loop()
    if (isr == true)
    {
      Serial.println(" --> Interrupt during event processing");
    }
  }
}

// Called then interrupt pin is set high
void interruptRoutine()
{
  isr = true;
  Serial.print("Interrupt! -- ");
}


