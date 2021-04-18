#include "State.h"

State::State(uint32_t dim) {
  if(dim == 0) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
				"Can not set zero-dimension state");
  }

  vals = std::vector<double>(dim);
}

State::State(const std::vector<double>& _vals) {
  if(_vals.size() == 0) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
				"Can not set zero-dimension state");
  }

  vals = _vals;
}

State::~State() {
}

uint32_t State::getDim() const {
  return vals.size();
}

double State::norm() const {
  return std::sqrt(dot(*this));
}

double State::dot(const State& other) const {
  if(vals.size() != other.vals.size()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
				"Can not calculate because dimension is different from");
  }

  double ret = 0;
  for(size_t i = 0; i < vals.size(); i++) {
    ret += vals[i] * other.vals[i];
  }

  return ret;
}

double State::distanceFrom(const State& other) const {
  return (other - *this).norm();
}

State State::normalized() const {
  return *this / norm();
}

bool State::isZero() const {
  for(const auto& val : vals) {
    if(val != 0) {
      return false;
    }
  }

  return true;
}

State State::operator +() const {
  return *this;
}

State State::operator -() const {
  State ret = *this;
  for(auto& val : ret.vals) {
    val *= -1;
  }

  return ret;
}

State State::operator +(const State& other) const {
  if(vals.size() != other.vals.size()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
				"Can not calculate because dimension is different from");
  }

  State ret = *this;
  for(size_t i = 0; i < ret.getDim(); i++) {
    ret.vals[i] += other.vals[i];
  }

  return ret;
}

State State::operator -(const State& other) const {
  if(vals.size() != other.vals.size()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
				"Can not calculate because dimension is different from");
  }

  State ret = *this;
  for(size_t i = 0; i < ret.getDim(); i++) {
    ret.vals[i] -= other.vals[i];
  }

  return ret;
}

bool State::operator ==(const State& other) const {
  if(vals.size() != other.vals.size()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
				"Can not calculate because dimension is different from");
  }

  for(size_t i = 0; i < getDim(); i++) {
    if(vals[i] != other.vals[i]) {
      return false;
    }
  }

  return true;
}

bool State::operator !=(const State& other) const {
  if(vals.size() != other.vals.size()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
				"Can not calculate because dimension is different from");
  }

  for(size_t i = 0; i < getDim(); i++) {
    if(vals[i] != other.vals[i]) {
      return true;
    }
  }

  return false;
}

State State::operator *(double s) const {
  State ret = *this;
  for(size_t i = 0; i < ret.getDim(); i++) {
    ret.vals[i] *= s;
  }

  return ret;
}

State State::operator /(double s) const {
  State ret = *this;
  for(size_t i = 0; i < ret.getDim(); i++) {
    ret.vals[i] /= s;
  }

  return ret;
}

std::ostream& operator << (std::ostream& os, const State& obj) {
  for(size_t i = 0; i < obj.getDim(); i++) {
    os << "[" << i << "] " << obj.vals[i];
    if(i != obj.getDim()) {
      os << ", ";
    }
  }
  return os;
};
