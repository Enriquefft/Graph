#ifndef BASE_GRAPH_HPP
#define BASE_GRAPH_HPP

enum class Representation { EdgeList, AdjacencyList, AdjacencyMatrix };

struct GraphConfig {
  bool is_weighted;
  bool is_directed;
  bool is_simple;
  Representation representation;
};

template <typename T, GraphConfig config> class BaseGraph {};

#endif // !BASE_GRAPH_HPP
