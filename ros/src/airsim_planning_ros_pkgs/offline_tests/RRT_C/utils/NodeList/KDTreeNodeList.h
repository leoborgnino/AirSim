#ifndef LIB_INCLUDE_NODE_KDTREENODELIST_H_
#define LIB_INCLUDE_NODE_KDTREENODELIST_H_

#include <NodeListBase.h>
#include <functional>
#include <numeric>
#include <algorithm>

class KDTreeNodeList : public NodeListBase {
  using NodePtr = std::shared_ptr<Node>;
public:
  explicit KDTreeNodeList(const uint32_t& dim);
  ~KDTreeNodeList();
  void add(const NodePtr& node);
  void init();
  int  getSize();
  NodePtr searchNN(const NodePtr& node);
  std::vector<NodePtr> searchNBHD(const NodePtr& node,
				  const double&  radius);
  std::vector<NodePtr> searchLeafs();
private:
  const double REBALANCE_RATIO = 0.1;

  struct KDTreeNode {
    int idx;
    int axis;
    std::shared_ptr<KDTreeNode> child_r;
    std::shared_ptr<KDTreeNode> child_l;
    KDTreeNode() : idx(-1), axis(-1), child_r(nullptr), child_l(nullptr) { }
  };

  using KDNodePtr = std::shared_ptr<KDTreeNode>;

  KDNodePtr            root_;
  std::vector<NodePtr> nodes_;
  int                  depth_;

  void clearRec(const KDNodePtr& node);

  KDNodePtr buildRec(std::vector<int>& indices,
		     const int&        offset,
		     const int&        npoints,
		     const int&        depth);

  KDNodePtr insertRec(const KDNodePtr& root,
		      const int&       new_node_index,
		      const int&       depth);

  void searchNNRec(const NodePtr&  query,
		   const KDNodePtr node,
		   NodePtr&        guess,
		   double&         min_dist) const;

  void searchNBHDRec(const NodePtr&        query,
		     const KDNodePtr       node,
		     std::vector<NodePtr>& near_nodes,
		     const double&         radius) const;
};

#endif /* LIB_INCLUDE_NODE_KDTREENODELIST_H_ */
