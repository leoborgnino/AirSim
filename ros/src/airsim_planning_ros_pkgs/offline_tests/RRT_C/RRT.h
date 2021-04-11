#ifndef LIB_INCLUDE_PLANNER_RRT_RRT_H_
#define LIB_INCLUDE_PLANNER_RRT_RRT_H_

#include <iostream>
#include <vector>
#include <random>
#include <cstdint>
#include <cmath>
#include <functional>
#include <PlannerBase.h>
#include <KDTreeNodeList.h>

class RRT : public PlannerBase {
    public:
        RRT(const uint32_t& dim,
            const uint32_t& max_sampling_num   = 10000,
            const double&   goal_sampling_rate = 0.05,
            const double&   expand_dist        = 1.0);
        ~RRT();

        void setMaxSamplingNum(uint32_t max_sampling_num) noexcept;
        void setGoalSamplingRate(double goal_sampling_rate);
        void setExpandDist(double expand_dist) noexcept;

        bool solve(const State& start, const State& goal) override;

    private:
        uint32_t max_sampling_num_;
        double   goal_sampling_rate_;
        double   expand_dist_;
    };

#endif /* LIB_INCLUDE_PLANNER_RRT_RRT_H_ */
