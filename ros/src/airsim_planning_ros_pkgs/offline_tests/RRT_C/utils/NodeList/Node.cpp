#include "Node.h"

// Constructor
Node::Node(const State&                _state,
	   const std::shared_ptr<Node> _parent,
	   const double&               _cost,
	   const double&               _cost_to_goal) :
  state(_state), parent(_parent), cost(_cost), cost_to_goal(_cost_to_goal), is_leaf(true) { }

// Destructor
Node::~Node() { }
