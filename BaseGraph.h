#ifndef GRAPH_IMPLEMENTATION_H
#define GRAPH_IMPLEMENTATION_H

#include <stdexcept>
#include <unordered_map>
#include <vector>

template <typename vertex_t, bool weighted, bool directed> //
class BaseGraph {

protected:
  // conditional bool or unsigned int
  using weight_t = typename std::conditional_t<weighted, bool, unsigned int>;

  std::unordered_map<vertex_t, unsigned int> m_vertices;
  std::unordered_map<unsigned int, vertex_t> lookup_table;
  std::vector<std::vector<weight_t>> m_adjMatrix;

public:
  bool isVertex(const vertex_t &vertex) const;
  bool isEdge(const vertex_t &fromV, const vertex_t &toV) const; // isVertex()

  void addVertex(const vertex_t &vertex); // isVertex() | UPDATE LOOKUP

  auto vertexHasEdges(const vertex_t &vertex) const -> bool; // isVertex()

  void removeVertex(const vertex_t &vertex,
                    const bool &force = false);                // UPDATE LOOKUP
  void removeEdge(const vertex_t &fromV, const vertex_t &toV); // UPDATE LOOKUP

  [[nodiscard]] auto getVertexCount() const -> unsigned int;
  [[nodiscard]] auto getEdgeCount() const -> unsigned int;

  auto getAdjacentVertices(const vertex_t &vertex) const
      -> std::vector<vertex_t>;

  auto getVertices() const -> std::vector<vertex_t>;
  auto getShortestPath(const vertex_t &fromV, const vertex_t &toV) const
      -> std::vector<vertex_t>;

  [[nodiscard]] auto getAdjacencyMatrix() const
      -> std::vector<std::vector<weight_t>>;
};

#endif // !GRAPH_IMPLEMENTATION_H
