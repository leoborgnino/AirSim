#ifndef LIB_INCLUDE_EUCLIDEANSPACE_EUCLIDEANSPACE_H_
#define LIB_INCLUDE_EUCLIDEANSPACE_EUCLIDEANSPACE_H_

#include <iostream>
#include <vector>
#include <limits>
#include <cstdint>

/**
 *  Express bound in euclidean space
 */
class Bound {
public:
  double low;
  double high;

  Bound();
  Bound(const double& low, const double& high);

  ~Bound();

  double getRange() const;
};

/**
 *  Express euclidean space
 */
class EuclideanSpace {
public:
  explicit EuclideanSpace(const uint32_t& dim);

  ~EuclideanSpace();

  uint32_t getDim() const noexcept;

  void setBound(std::vector<Bound>& bounds);

  Bound getBound(const uint32_t& dim) const;

  const std::vector<Bound>& getBoundsRef() const;

private:
  uint32_t dim_;
  std::vector<Bound> bounds_;
};
#endif /* LIB_INCLUDE_EUCLIDEANSPACE_EUCLIDEANSPACE_H_ */
