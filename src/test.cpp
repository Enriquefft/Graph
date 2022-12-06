#include "Graph.h"
#include <iostream>
#include <vector>

using std::vector;

template <typename edge, typename vertex>
void printIndexedMatrix(const vector<vector<edge>> &matrix,
                        const vector<vertex> &indexes) {

  if (matrix.size() != indexes.size()) {
    std::cout << "Matrix and indexes have different sizes" << std::endl;
    return;
  }

  std::cout << "  ";
  for (const auto &index : indexes) {
    std::cout << "\t" << index;
  }

  for (size_t i = 0; i < matrix.size(); ++i) {
    if (matrix[i].size() != indexes.size()) {
      std::cout << "Matrix and indexes have different sizes" << std::endl;
      return;
    }
    std::cout << std::endl << indexes[i];
    for (size_t j = 0; j < matrix[i].size(); ++j) {
      std::cout << "\t" << matrix[i][j];
    }
  }
}

template <typename index, typename value>
void printIndexedVector(const vector<value> &values,
                        const vector<index> &indexes) {
  if (values.size() != indexes.size()) {
    std::cout << "Values and indexes have different sizes" << std::endl;
    return;
  }
  for (size_t i = 0; i < values.size(); ++i) {
    std::cout << indexes[i] << ":\t" << values[i] << std::endl;
  }
}

template <typename vertex>
void printIndexedVector(const vector<std::pair<unsigned int, vertex>> &values) {
  for (const auto &value : values) {
    std::cout << value.second << ":\t" << value.first << std::endl;
  }
}

template <typename vertex> void printVector(const vector<vertex> &values) {
  for (const auto &value : values) {
    std::cout << value << std::endl;
  }
}

int main() {
  Graph<char, true> graph;
  graph.addEdge('a', 'c', 1);
  graph.addEdge('c', 'd', 2);
  graph.addEdge('d', 'e', 3);
  graph.addEdge('e', 'b', 1);
  graph.addEdge('a', 'b', 1);

  graph.addVertex('x');

  printIndexedMatrix(graph.getAdjacencyMatrix(), graph.getVertices());
  std::cout << std::endl << std::endl;
  printVector(graph.getShortestPath('a', 'x'));
}
