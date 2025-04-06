#ifndef LOW_PASS_FILTER_HPP
#define LOW_PASS_FILTER_HPP

class LowPassFilter
{
public:
  LowPassFilter(float alpha);   // Constructor with alpha value
  float update(float newValue); // Update filter with a new value (should be called ONCE every time for new data)
  float getValue() const;       // Get the current filtered value

private:
  float alpha;         // Smoothing factor
  float filteredValue; // Last filtered value
  bool initialized;    // Check if initialized
};

#endif
