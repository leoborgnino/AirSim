#include "EuclideanSpace.h"

Bound::Bound() :
  low(0), high(0) {
}

Bound::Bound(const double& _low, const double& _high) {
  if(_high < _low) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
				"Bound is invalid");
  }

  high = _high;
  low  = _low;
}

Bound::~Bound() {
}

double Bound::getRange() const {
  return high - low;
}

EuclideanSpace::EuclideanSpace(const uint32_t& dim) {
  if(dim == 0) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
				"Dimension is invalid");
  }
  dim_    = dim;
  bounds_ = std::vector<Bound>(dim, Bound(0, 0));
}

EuclideanSpace::~EuclideanSpace() {
}

uint32_t EuclideanSpace::getDim() const noexcept {
  return dim_;
}

void EuclideanSpace::setBound(std::vector<Bound>& bounds) {
  if(bounds.size() != dim_) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
				"Condition of bound is invalid");
  }
  bounds_ = bounds;
}

Bound EuclideanSpace::getBound(const uint32_t& dim) const {
  if(!(0 < dim && dim <= dim_)) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
				"Dimention is out of range");
  }
  return bounds_[dim - 1];
}

const std::vector<Bound>& EuclideanSpace::getBoundsRef() const {
  return bounds_;
}
