#ifndef LIB_INCLUDE_STATE_STATE_H_
#define LIB_INCLUDE_STATE_STATE_H_

#include <iostream>
#include <memory>
#include <vector>
#include <cstdint>
#include <cmath>

/**
 *  Representación y operación de Estados Multidimensionales
 */
class State {
public:
  std::vector<double> vals;

  explicit State(uint32_t dim);

  explicit State(const std::vector<double>& _vals);

  template<class... A>
  State(const A&... _vals) :
    vals(std::initializer_list<double>{_vals...}) {
  }

  ~State();

  uint32_t getDim() const;

  double norm() const;

  double dot(const State& other) const;

  double distanceFrom(const State& other) const;

  State normalized() const;

  bool isZero() const;

  State operator +() const;
  State operator -() const;
  State operator +(const State& other) const;
  State operator -(const State& other) const;
  bool  operator ==(const State& other) const;
  bool  operator !=(const State& other) const;
  State operator *(double s) const;
  State operator /(double s) const;

  friend std::ostream& operator << (std::ostream& os, const State& obj);
};

#endif /* LIB_INCLUDE_STATE_STATE_H_ */
