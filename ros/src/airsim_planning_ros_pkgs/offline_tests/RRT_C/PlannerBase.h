#ifndef LIB_INCLUDE_PLANNER_PLANNERBASE_H_
#define LIB_INCLUDE_PLANNER_PLANNERBASE_H_

#include <NodeListBase.h>
#include <ConstraintBase.h>
#include <Sampler.h>

/**
 *  Base class of planners which are sampling-based method
 */
class PlannerBase {
 public:
  explicit PlannerBase(const uint32_t& dim,
		       std::shared_ptr<NodeListBase> node_list);
  virtual ~PlannerBase();

  PlannerBase(const PlannerBase&) = delete;
  PlannerBase& operator =(const PlannerBase&) = delete;
  PlannerBase(PlannerBase&&) noexcept = delete;
  PlannerBase& operator =(PlannerBase&&) noexcept = delete;

  /**
   *  Execute path planning
   *  @start:  start state
   *  @goal:   goal state
   *  @Return: whether the path planning was successful
   */
  virtual bool solve(const State& start,
		     const State& goal) = 0;

  void setProblemDefinition(const std::shared_ptr<ConstraintBase>& constraint);

  void setTerminateSearchCost(const double& terminate_search_cost);

  const std::vector<State>& getResult() const;

  double getResultCost() const;

  std::shared_ptr<NodeListBase> getNodeList() const;

 protected:
  std::vector<State>              result_;
  double                          result_cost_;
  double                          terminate_search_cost_;
  std::shared_ptr<ConstraintBase> constraint_;
  std::shared_ptr<NodeListBase>   node_list_;
  std::unique_ptr<Sampler>        sampler_;

  /**
   *  Generate Steered node that is 'expand_dist' away from 'src_node' to 'dst_node' direction
   *  @src_node:    source node
   *  @dst_node:    destination node
   *  @expand_dist: distance from 'src_node' of steered node
   *  @Return:      steered node
   */
  std::shared_ptr<Node> generateSteerNode(const std::shared_ptr<Node>& src_node,
					  const std::shared_ptr<Node>& dst_node,
					  const double&                expand_dist) const;

  /**
   *  Choose parent node from near node that find in findNearNodes()
   *  @target_node:       target node
   *  @node_list:         list that contein existing node
   *  @near_node_indexes: return value of findNearNodes()
   *  @Return: node that choosed new parent node
   */
  void updateParent(const std::shared_ptr<Node>&              target_node,
		    const std::vector<std::shared_ptr<Node>>& near_nodes) const;

  /**
   *  redefine parent node of near node that find in findNearNodes()
   *  @node_list:         list that contein existing node
   *  @near_node_indexes: return value of findNearNodes()
   *  @Return:            nodes which are rewired
   */
  std::vector<std::shared_ptr<Node>> rewireNearNodes(std::shared_ptr<Node>&              new_node,
						     std::vector<std::shared_ptr<Node>>& near_nodes) const;

};

#endif /* LIB_INCLUDE_PLANNER_PLANNERBASE_H_ */
