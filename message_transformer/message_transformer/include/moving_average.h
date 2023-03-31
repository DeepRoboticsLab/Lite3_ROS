#ifndef MOVING_AVERAGE__H
#define MOVING_AVERAGE__H

#include <vector>

class MovingAverage {
 private:
  const int filter_length;
  bool filled;
  int index;
  double sum, average;
  std::vector<double> data;

 public:
  /**
   * Creates a new instance of MovingAverage with
   * given filter length.
   */
  MovingAverage(const int&);
  /**
   * Releases the memory objects associated with the
   * current MovingAverage instance.
   */
  ~MovingAverage();
  /**
   * Adds a new element in the Moving Average vector.
   * Updates the current average.
   */
  void in(const double&);
  /**
   * Returns the current average as update after the invocation
   * of MovingAverage::add(double).
   */
  double out() const;
};
#endif