#include "Graph.h"
#include <iostream>

int main() {
  Graph<char, true> graph;
  graph.addEdge('a', 'b', 20);
  graph.addEdge('a', 'c', 1);
  graph.addEdge('c', 'd', 2);
  graph.addEdge('d', 'e', 3);
  graph.addEdge('e', 'b', 4);

  auto path = graph.getCheapestPath('a', 'b');

  for (auto node : path) {
    std::cout << "weight:\t" << node.first << "\tvalue:\t" << node.second
              << std::endl;
  }
}
