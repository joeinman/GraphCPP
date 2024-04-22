#include <iostream>
#include <GraphCPP/GraphCPP.h>

int main()
{
    GraphCPP::DirectedGraph<int> graph;

    graph.addNode(1);
    graph.addNode(2);
    graph.addNode(3);

    graph.addEdge(1, 2, 1);
    graph.addEdge(2, 3, 1);
    graph.addEdge(3, 1, 1);

    // Get Neighbours
    std::cout << "Neighbours of 1: ";
    for (auto &neighbour : graph.getNeighbors(1))
    {
        std::cout << neighbour << " ";
    }
}