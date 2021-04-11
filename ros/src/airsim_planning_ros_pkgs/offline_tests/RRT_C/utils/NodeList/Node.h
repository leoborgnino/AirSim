#ifndef LIB_INCLUDE_NODE_NODE_H_
#define LIB_INCLUDE_NODE_NODE_H_

#include <State.h>
#include <memory>
#include <limits>

// Clase Nodo

class Node {
public:
  State                 state;
  std::shared_ptr<Node> parent;
  double                cost;
  double                cost_to_goal;
  bool                  is_leaf;
  
  Node(const State&                _state,
       const std::shared_ptr<Node> _parent,
       const double&               _cost = 0.0,
       const double&               _cost_to_goal = std::numeric_limits<double>::max());
  
  ~Node();
  
};

#endif /* LIB_INCLUDE_NODE_NODE_H_ */
