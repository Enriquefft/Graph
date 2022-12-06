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
  graph.addEdge('A', 'B', 5);
  graph.addEdge('A', 'C', 7);
  graph.addEdge('B', 'C', 9);

  graph.addEdge('C', 'D', 8);
  graph.addEdge('C', 'E', 7);
  graph.addEdge('E', 'D', 5);

  graph.addEdge('B', 'E', 15);
  graph.addEdge('B', 'F', 6);

  graph.addEdge('E', 'F', 8);
  graph.addEdge('E', 'G', 9);
  graph.addEdge('G', 'F', 11);

  auto met = graph.getMinimumExpansionTree();

  for (const auto &edge : met) {
    std::cout << std::get<0>(edge) << " - " << std::get<1>(edge) << " : "
              << std::get<2>(edge) << std::endl;
  }
}
