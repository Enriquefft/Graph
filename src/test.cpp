#include "Graph.hpp"
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

  Graph<char, true, false> graph;

  graph.addEdge('A', 'B', 2);
  graph.addEdge('A', 'G', 3);
  graph.addEdge('B', 'G', 6);

  graph.addEdge('A', 'F', 7);
  graph.addEdge('F', 'I', 5);
  graph.addEdge('F', 'E', 6);

  graph.addEdge('B', 'C', 4);
  graph.addEdge('C', 'H', 2);
  graph.addEdge('C', 'D', 2);

  graph.addEdge('G', 'I', 1);
  graph.addEdge('G', 'H', 3);
  graph.addEdge('I', 'H', 4);

  graph.addEdge('I', 'E', 2);
  graph.addEdge('H', 'D', 8);
  graph.addEdge('E', 'D', 1);

  auto met = graph.getMinimumExpansionTree();
  // auto cheap = graph.getCheapestPath('A', 'G');
  //
  // for (const auto &node : cheap) {
  //   std::cout << node.second << " ";
  // }
  // std::cout << std::endl;

  for (const auto &edge : met) {
    std::cout << std::get<0>(edge) << " - " << std::get<1>(edge) << " : "
              << std::get<2>(edge) << std::endl;
  }
}
