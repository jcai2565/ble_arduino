#ifndef LED_HPP
#define LED_HPP

#include <Arduino.h>

// Function Declarations
void ledSetup();
void ledStartBlink(int interval);
void ledStopBlink();
void ledCheckAndSet();

#endif // LED_HPP
