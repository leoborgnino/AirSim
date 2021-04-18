#ifndef LIB_INCLUDE_NODE_NODELISTBASE_H_
#define LIB_INCLUDE_NODE_NODELISTBASE_H_

#include <Node.h>

/**
 *  Base class of node list for sampling-based planners
 */
class NodeListBase {
  using NodePtr = std::shared_ptr<Node>;
public:
  const uint32_t DIM;
  explicit NodeListBase(const uint32_t& _dim);
  virtual ~NodeListBase();
  virtual void add(const NodePtr& node) = 0;
  virtual void init() = 0;
  virtual int getSize() = 0;
  virtual NodePtr searchNN(const NodePtr& node) = 0;
  virtual std::vector<NodePtr> searchNBHD(const NodePtr& node,
					  const double&  radius) = 0;
  virtual std::vector<NodePtr> searchLeafs() = 0;
};

#endif /* LIB_INCLUDE_NODE_NODELISTBASE_H_ */
