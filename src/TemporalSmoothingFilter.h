#include <deque>
#include <algorithm>
#include <numeric>
#include <stdexcept>

enum FilterMethod{
  Median,
  Average,
};

/// Hey, thats only defined for doubles and for vectors!
template<class T>
class TemporalSmoothingFilter{
  size_t m_timewindow;
  std::deque<T> m_buffer;
  T m_zeroValue;
  FilterMethod m_method;  
  
  public:
  
  TemporalSmoothingFilter(size_t timewindow, 
                          const FilterMethod &method,
                          const T &zeroValue=T(0)) throw (std::logic_error);
  void clear();
  T push(const T &t);
};
