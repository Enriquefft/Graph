#include "BaseGraph.h"

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

  auto getEdgeList() -> std::vector<std::pair<vertex_t, vertex_t>>;
  void addEdge(const vertex_t &from, const vertex_t &toV);
};

template <typename vertex_t, bool directed>
class Graph<vertex_t, true, directed>
    : public BaseGraph<vertex_t, false, directed> {

  // easy acces to parent class members
  using BaseGraph<vertex_t, false, directed>::m_vertices;
  using BaseGraph<vertex_t, false, directed>::m_adjMatrix;
  using BaseGraph<vertex_t, false, directed>::lookup_table;

public:
  using BaseGraph<vertex_t, false, directed>::addVertex;
  using BaseGraph<vertex_t, false, directed>::isVertex;
  using BaseGraph<vertex_t, false, directed>::getAdjacentVertices;

  auto getEdgeList()
      -> std::vector<std::tuple<vertex_t, vertex_t, unsigned int>>;
  void addEdge(const vertex_t &from, const vertex_t &toV,
               const unsigned int &weight);

  auto getCheapestPath(const vertex_t &fromV, const vertex_t &toV) const
      -> std::vector<std::pair<unsigned int, vertex_t>>;
};
