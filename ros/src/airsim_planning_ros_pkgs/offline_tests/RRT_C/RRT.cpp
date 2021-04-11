#include <RRT.h>

RRT::RRT(const uint32_t& dim,
	 const uint32_t& max_sampling_num,
	 const double&   goal_sampling_rate,
	 const double&   expand_dist       ) :
  PlannerBase(dim, std::make_shared<KDTreeNodeList>(dim)),
  max_sampling_num_(max_sampling_num),
  expand_dist_(expand_dist) {
  setGoalSamplingRate(goal_sampling_rate);
  }

RRT::~RRT() {
}

void RRT::setMaxSamplingNum(uint32_t max_sampling_num) noexcept {
  max_sampling_num_ = max_sampling_num;
}

void RRT::setGoalSamplingRate(double goal_sampling_rate) {
  if(!(0.0 <= goal_sampling_rate && goal_sampling_rate <= 1.0)) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
				"Rate of Sampling goal state is invalid");
  }

  goal_sampling_rate_ = goal_sampling_rate;
}

void RRT::setExpandDist(double expand_dist) noexcept {
  expand_dist_ = expand_dist;
}

bool RRT::solve(const State& start, const State& goal) {
  // initialize list of node
  node_list_->init();
  node_list_->add(std::make_shared<Node>(start, nullptr));

  // sampling on euclidean space
  uint32_t sampling_cnt = 0;
  std::shared_ptr<Node> end_node;
  while(true) {
    auto rand_node = std::make_shared<Node>(goal, nullptr);
    if(goal_sampling_rate_ < sampler_->getUniformUnitRandomVal()) {
      rand_node->state = sampler_->run(Sampler::Mode::WholeArea);

      // resample when node dose not meet constraint
      if(constraint_->checkConstraintType(rand_node->state) == ConstraintType::NOENTRY) {
	continue;
      }
    }

    // get index of node that nearest node from sampling node
    auto nearest_node = node_list_->searchNN(rand_node);

    // generate new node
    auto new_node = generateSteerNode(nearest_node, rand_node, expand_dist_);

    // add to list if new node meets constraint
    if(constraint_->checkCollision(nearest_node->state, new_node->state)) {
      node_list_->add(new_node);

      // terminate processing if distance between new node and goal state is less than 'expand_dist'
      if(new_node->state.distanceFrom(goal) <= expand_dist_) {
	end_node = std::make_shared<Node>(goal, new_node);
	node_list_->add(end_node);
	break;
      }
    }

    sampling_cnt++;
    if(max_sampling_num_ == sampling_cnt) {
      return false;
    }
  }

  // store the result
  result_.clear();
  auto cost = 0.0;
  while(true) {
    result_.insert(result_.begin(), end_node->state);
    if(end_node->parent == nullptr) {
      cost += end_node->state.distanceFrom(start);
      break;
    }
    else {
      cost += end_node->state.distanceFrom(end_node->parent->state);
    }

    end_node = end_node->parent;
  }

  result_cost_ = cost;
  return true;
}
