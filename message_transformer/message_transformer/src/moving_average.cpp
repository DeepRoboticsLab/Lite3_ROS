#include "moving_average.h"

/// @brief Creates a new instance of MovingAverage with given filter length.
/// @param filterLength filter length
MovingAverage::MovingAverage(const int& filterLength)
    : filter_length(filterLength),
      filled(false),
      index(-1),
      sum(0),
      average(0) {
  data.resize(filter_length, 0);
}


/// @brief  Releases the memory objects associated with the current MovingAverage instance.
MovingAverage::~MovingAverage() {}


/// @brief  Adds a new element in the Moving Average vector. Updates the current average.
/// @param x 
void MovingAverage::in(const double& x) {
  index = (index + 1) % filter_length;
  sum -= data[index];
  data[index] = x;
  sum += x;
  if (!filled && index == filter_length - 1) {
    filled = true;
  }
  if (filled) {
    average = sum / filter_length;
  } else {
    average = sum / (index + 1);
  }
}


/// @brief Returns the current average as update after the invocation of MovingAverage::add(double).
/// @return average data
double MovingAverage::out() const { return average; }