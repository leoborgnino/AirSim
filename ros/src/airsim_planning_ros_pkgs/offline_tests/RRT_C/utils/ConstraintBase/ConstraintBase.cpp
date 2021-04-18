#include "ConstraintBase.h"

ConstraintBase::ConstraintBase(const EuclideanSpace& _space) :
  space(_space) {
}

ConstraintBase::~ConstraintBase() {
}

uint32_t ConstraintBase::getDim() const {
  return space.getDim();
}

bool ConstraintBase::checkCollision(const State& src,
				    const State& dst) const {
  return (checkConstraintType(src) == ConstraintType::ENTAERABLE &&
	  checkConstraintType(dst) == ConstraintType::ENTAERABLE) ? true : false;
}

ConstraintType ConstraintBase::checkConstraintType(const State &state) const {
  for(size_t i = 0; i < state.getDim(); i++) {
    auto bound = space.getBound(i + 1);

    // return NOENTRY Type if the state is out of range
    if(state.vals[i] < bound.low || bound.high < state.vals[i]) {
      return ConstraintType::NOENTRY;
    }
  }

  return ConstraintType::ENTAERABLE;
}
