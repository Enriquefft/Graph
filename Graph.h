#include "BaseGraph.h"

// Generic Graph
template <typename vertex_t, bool weighted>
class Graph : BaseGraph<vertex_t, weighted> {};

// specialization for unweighted graph
template <typename vertex_t>
class Graph<vertex_t, false> : public BaseGraph<vertex_t, false> {

  // easy acces to parent class members

  using BaseGraph<vertex_t, false>::m_vertices;
  using BaseGraph<vertex_t, false>::m_adjMatrix;
  using BaseGraph<vertex_t, false>::lookup_table;

public:
  using BaseGraph<vertex_t, false>::addVertex;

  auto getEdgeList() -> std::vector<std::pair<vertex_t, vertex_t>>;
  void addEdge(const vertex_t &from, const vertex_t &toV);
};
