#include "lpf.hpp"

LowPassFilter::LowPassFilter(float alpha) : alpha(alpha), initialized(false) {}

float LowPassFilter::update(float newValue)
{
  if (!initialized)
  {
    filteredValue = newValue; // Initialize with first value
    initialized = true;
  }
  else
  {
    filteredValue = alpha * newValue + (1 - alpha) * filteredValue;
  }
  return filteredValue;
}

float LowPassFilter::getValue() const
{
  return filteredValue;
}
