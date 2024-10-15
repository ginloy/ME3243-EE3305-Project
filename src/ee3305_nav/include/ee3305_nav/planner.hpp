#include <iostream>
#include <iomanip>
#include <cmath>
#include <list>

#pragma once
namespace ee3305
{
    // =================================================================================================
    /** Node object stores the information needed for planner nodes. */
    struct PlannerNode
    {
        bool visited;
        int i;
        int j;
        double g;
        double f;
        PlannerNode *parent;

        explicit PlannerNode(int i, int j) 
            : visited(false), i(i), j(j), g(INFINITY), f(INFINITY), parent(nullptr) {}
    };

    // =================================================================================================
    /** OpenList manages the open list operations of the planner. */
    class OpenList
    {
    private:
        std::list<PlannerNode *> list;

    public:
        explicit OpenList() {}

        /** Queues the node into the open list */
        void queue(PlannerNode *const node)
        {
            auto it_node = list.begin(); // a queued node in the open list
            while (it_node != list.end())
            {
                const double &queued_f = (*it_node)->f; // f-cost of queued node in open list.
                if (node->f < queued_f)
                    break; // insert `node` before queued node if it is cheaper.

                it_node = std::next(it_node); // go to next queued node in open list.
            }
            list.insert(it_node, node);
        }

        /** Returns the node with the cheapest f cost */
        PlannerNode *poll()
        {
            PlannerNode *node = list.front();
            list.pop_front();
            return node;
        }

        /** Returns true if the open list is empty, false otherwise */
        bool empty() { return list.empty(); }
    };
}