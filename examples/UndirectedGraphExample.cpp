#include <iostream>
#include <GraphCPP/GraphCPP.h>

int main()
{
    GraphCPP::UndirectedGraph<char> graph;

    graph.addNode('a');
    graph.addNode('b');
    graph.addNode('c');
    graph.addNode('d');

    graph.addEdge('a', 'b', 1);
    graph.addEdge('b', 'c', 1);
    graph.addEdge('c', 'a', 1);
    graph.addEdge('a', 'd', 1);
    // graph.addEdge('d', 'c', 1);
    // graph.addEdge('d', 'b', 1);

    std::cout << "Fiedler Value: " << graph.getFiedlerValue() << std::endl;
    std::cout << "Spectral Gap: " << graph.getSpectralGap() << std::endl;
}