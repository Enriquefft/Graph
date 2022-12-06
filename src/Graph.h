#include "BaseGraph.h"
#include <algorithm>
#include <queue>
#include <unordered_map>

// Generic Graph interface
template <typename vertex_t, bool weighted = false, bool directed = false>
class Graph : BaseGraph<vertex_t, weighted, directed> {};

// specialization for unweighted graph
template <typename vertex_t, bool directed>
class Graph<vertex_t, false, directed>
    : public BaseGraph<vertex_t, false, directed> {

  // easy acces to parent class members
  using BaseGraph<vertex_t, false, directed>::m_vertices;
  using BaseGraph<vertex_t, false, directed>::m_adjMatrix;
  using BaseGraph<vertex_t, false, directed>::lookup_table;

public:
  using BaseGraph<vertex_t, false, directed>::addVertex;

  auto getEdgeList() const -> std::vector<std::pair<vertex_t, vertex_t>>;
  void addEdge(const vertex_t &from, const vertex_t &toV);
};

template <typename vertex_t, bool directed>
class Graph<vertex_t, true, directed>
    : public BaseGraph<vertex_t, true, directed> {

  // easy acces to parent class members
  using BaseGraph<vertex_t, true, directed>::m_vertices;
  using BaseGraph<vertex_t, true, directed>::m_adjMatrix;
  using BaseGraph<vertex_t, true, directed>::lookup_table;

  using WeightedEdgeList =
      std::vector<std::tuple<vertex_t, vertex_t, unsigned int>>;

  template <OutputType output_type = OutputType::EdgeList>
  triConditional_t<output_type, vertex_t, true>
  formatGraphOutput(const WeightedEdgeList &edgeList) const {

    if constexpr (output_type == OutputType::EdgeList) {
      return edgeList;
    } else if constexpr (output_type == OutputType::AdjacencyMatrix) {
      return m_adjMatrix;
    } else {
      std::vector<std::list<std::pair<unsigned int, vertex_t>>> AdjacencyList;
      return AdjacencyList;
    }
  }

public:
  using BaseGraph<vertex_t, true, directed>::addVertex;
  using BaseGraph<vertex_t, true, directed>::isVertex;
  using BaseGraph<vertex_t, true, directed>::getVertices;

  auto getEdgeList() const
      -> std::vector<std::tuple<vertex_t, vertex_t, unsigned int>>;

  void addEdge(const vertex_t &from, const vertex_t &toV,
               const unsigned int &weight);

  auto getCheapestPath(const vertex_t &fromV, const vertex_t &toV) const
      -> std::vector<std::pair<unsigned int, vertex_t>>;

  template <OutputType Type = OutputType::EdgeList>
  auto getMinimumExpansionTree() const
      -> triConditional_t<Type, vertex_t, true> {

    auto edgeList = getEdgeList();

    // sort edges by weight
    std::sort(edgeList.begin(), edgeList.end(), [](auto &left, auto &right) {
      return std::get<2>(left) < std::get<2>(right);
    });

    std::vector<std::tuple<vertex_t, vertex_t, unsigned int>> met;

    for (const auto &edge : edgeList) {

      std::unordered_map<vertex_t, bool> visited;

      // fill visited map with false
      std::for_each(met.begin(), met.end(), [&visited](auto &metEdge) {
        if (visited.contains(std::get<0>(metEdge))) {
          visited[std::get<0>(metEdge)] = true;
        }
        if (visited.contains(std::get<1>(metEdge))) {
          visited[std::get<1>(metEdge)] = true;
        }
      });

      std::queue<vertex_t> Vqueue;

      Vqueue.push(std::get<0>(edge));
      visited[std::get<0>(edge)] = true;

      bool pathExists = false;

      while (!Vqueue.empty()) {
        auto currentV = Vqueue.front();
        Vqueue.pop();

        if (currentV == std::get<1>(edge)) {
          pathExists = true;
          break;
        }

        for (const auto &adjV : met) {
          if (std::get<0>(adjV) == currentV && !visited[std::get<1>(adjV)]) {
            Vqueue.push(std::get<1>(adjV));
            visited[std::get<1>(adjV)] = true;
          } else if (std::get<1>(adjV) == currentV &&
                     !visited[std::get<0>(adjV)]) {
            Vqueue.push(std::get<0>(adjV));
            visited[std::get<0>(adjV)] = true;
          }
        }
      }
      if (!pathExists) {
        met.push_back(edge);
      }
    }

    triConditional_t<Type, vertex_t, true> return_value;

    switch (Type) {
    case OutputType::EdgeList:
      return_value = formatGraphOutput<Type>(met);
      break;
    case OutputType::AdjacencyMatrix:
      throw std::runtime_error("Adjacency Matrix Not implemented");
      break;
    // return getAdjacencyMatrix(met);
    case OutputType::AdjacencyList:
      throw std::runtime_error("Adjacency List Not implemented");
      break;
      // return getAdjacencyList(met);
    }
    return return_value;
  }
};
