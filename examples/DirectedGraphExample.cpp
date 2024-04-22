#include <iostream>
#include <GraphCPP/GraphCPP.h>

int main()
{
    GraphCPP::DirectedGraph<char> graph;

    graph.addNode('a');
    graph.addNode('b');
    graph.addNode('c');

    graph.addEdge('a', 'b', 1);
    graph.addEdge('b', 'c', 2);
    graph.addEdge('c', 'a', 3);

    std::vector<char> neighbors = graph.getNeighbors('a');
    for (const char &neighbor : neighbors)
        std::cout << neighbor << std::endl;
}