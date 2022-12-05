#include "Graph.h"
#include <iostream>

int main() {
  Graph<int, false> graph = Graph<int, false>();
  graph.addVertex(1);
  graph.addVertex(2);
  graph.addEdge(1, 3);

  auto vertex = graph.getVertices();
  auto mat = graph.getAdjacencyMatrix();

  for (auto vert : vertex) {
    std::cout << vert << std::endl;
  }

  for (const auto &row : mat) {
    for (const auto &col : row) {
      std::cout << col << ' ';
    }
    std::cout << '\n';
  }
}
