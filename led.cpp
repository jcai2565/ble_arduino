#include "led.hpp"

// Global variables to track LED state
const static int ledPin = LED_BUILTIN;
static int ledState = LOW;
static int ledInterval = 0; // Interval in milliseconds. ledInterval == 0 <=> LED should be off.
static unsigned long ledPreviousMillis = 0;


// LED FUNCTIONS

void ledSetup(){
  pinMode(ledPin, OUTPUT); //Initialize LED pin
  ledStartBlink(500);
}

void ledStartBlink(int i){
  ledInterval = i;
  ledPreviousMillis = 0;
}

void ledStopBlink(){
  ledInterval = 0;
  digitalWrite(ledPin, LOW);
}

void ledCheckAndSet(){
  // To be called in loop(): handles checking LED state and turning it on/off to blink.
  // Not sure why calling it in central loop makes it fail to blink? 

  // [ledInterval] == 0 if the LED should not blink.
  if (ledInterval != 0){
    unsigned long currentMillis = millis();
    if (currentMillis - ledPreviousMillis >= ledInterval){
      ledPreviousMillis = currentMillis;

      //toggle state
      if (ledState == HIGH){
        ledState = LOW;
      } else {
        ledState = HIGH;
      }
    }
    digitalWrite(ledPin, ledState);
    
    // Five seconds after startup, stop blinking
    if (currentMillis > 5000){
      ledStopBlink();
    }

  }
  //DEBUG: when in central loop ledCheckAndSet() is called but setting the ledInterval does not cause the LED to start blinking again...
  // Serial.println("ledcheck called");
}
