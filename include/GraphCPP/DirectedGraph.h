#pragma once

#include "BaseGraph.h"

namespace GraphCPP
{

    template <typename NodeType>
    class DirectedGraph : public BaseGraph<NodeType, int>
    {
    public:
        void addNode(const NodeType &vertex) override
        {
            adjacencyList[vertex] = std::unordered_map<NodeType, int>();
        }

        void addEdge(const NodeType &from, const NodeType &to, const int &weight) override
        {
            adjacencyList[from][to] = weight;
        }

        void removeNode(const NodeType &node) override
        {
            adjacencyList.erase(node);
            for (auto &pair : adjacencyList)
            {
                pair.second.erase(node);
            }
        }

        void removeEdge(const NodeType &from, const NodeType &to) override
        {
            adjacencyList[from].erase(to);
        }
    };

}