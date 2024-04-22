#pragma once

#include <unordered_map>
#include <vector>

namespace GraphCPP
{

    template <typename NodeType, typename EdgeType>
    class BaseGraph
    {
    protected:
        std::unordered_map<NodeType, std::unordered_map<NodeType, EdgeType>> adjacencyList;

    public:
        virtual void addNode(const NodeType &vertex) = 0;
        virtual void addEdge(const NodeType &from, const NodeType &to, const EdgeType &weight) = 0;

        virtual void removeNode(const NodeType &vertex) = 0;
        virtual void removeEdge(const NodeType &from, const NodeType &to) = 0;

        std::vector<NodeType> getNeighbors(const NodeType &vertex)
        {
            std::vector<NodeType> neighbors;
            if (adjacencyList.find(vertex) != adjacencyList.end())
            {
                for (const auto &pair : adjacencyList.at(vertex))
                {
                    neighbors.push_back(pair.first);
                }
            }
            return neighbors;
        }

        virtual ~BaseGraph() {}
    };

}