#ifndef LIB_INCLUDE_CONSTRAINT_CONSTRAINT_H_
#define LIB_INCLUDE_CONSTRAINT_CONSTRAINT_H_

#include <cstdint>

#include <EuclideanSpace.h>
#include <State.h>

/**
 *  definition of type of constraint
 */
enum class ConstraintType {
  NOENTRY,
  ENTAERABLE
};

/**
 *  Base class of constraint for planner
 */
class ConstraintBase {
public:
  EuclideanSpace space;

  /**
   *  Constructor(ConstraintBase)
   *  @space: target space
   */
  explicit ConstraintBase(const EuclideanSpace& _space);
  virtual ~ConstraintBase();

  uint32_t getDim() const;

  /**
   *  Check whether collision occurred between src and dst
   *  @src:    source state
   *  @dst:    destination state
   *  @Return: If the path of between 'src' and 'dst' entry ConstraintType::NOENTRY,
   *           return false
   */
  virtual bool checkCollision(const State& src,
			      const State& dst) const;

  /**
   *  Check constraint at given state
   *  @state: target state
   *  @Return: type of constraint at target state
   */
  virtual ConstraintType checkConstraintType(const State& state) const;
};

#endif /* LIB_INCLUDE_CONSTRAINT_CONSTRAINT_H_ */
