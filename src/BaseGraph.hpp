#ifndef GRAPH_IMPLEMENTATION_H
#define GRAPH_IMPLEMENTATION_H

#include <list>
#include <stdexcept>
#include <unordered_map>
#include <vector>

// enum for output type
enum class OutputType : unsigned int {
  EdgeList = 0,
  AdjacencyMatrix = 1,
  AdjacencyList = 2
};

template <OutputType Type, typename vertex_t, bool weighted>
struct triConditional;

template <typename vertex_t, bool weighted>
struct triConditional<OutputType::EdgeList, vertex_t, weighted> {

  using edgeType =
      std::conditional_t<weighted, std::tuple<vertex_t, vertex_t, unsigned int>,
                         std::pair<vertex_t, vertex_t>>;

  using type = std::vector<edgeType>;
};

template <typename vertex_t, bool weighted>
struct triConditional<OutputType::AdjacencyList, vertex_t, weighted> {

  using edgeType =
      std::conditional_t<weighted, std::pair<unsigned int, vertex_t>, vertex_t>;

  using type = std::vector<std::list<edgeType>>;
};

template <typename vertex_t, bool weighted>
struct triConditional<OutputType::AdjacencyMatrix, vertex_t, weighted> {
  using edgeType = std::conditional_t<weighted, unsigned int, bool>;
  using type = std::vector<std::vector<edgeType>>;
};

template <OutputType Type, typename vertex_t, bool weighted>
using triConditional_t =
    typename triConditional<Type, vertex_t, weighted>::type;

template <typename vertex_t, bool weighted, bool directed> //
class BaseGraph {

protected:
  // conditional bool or unsigned int
  using weight_t = typename std::conditional_t<weighted, unsigned int, bool>;

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
