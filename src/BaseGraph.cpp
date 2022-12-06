#include <algorithm>
#include <queue>
#include <stdexcept>

#include "../../utils/utils.h"
#include "BaseGraph.h"

using std::vector;
using utils::unsignedCast;

/*
  #################################################################
  ################# GENERIC GRAPH IMPLEMENTATION ##################
  #################################################################
*/

// ################ IS VERTEX ##################
template <typename vertex_t, bool weighted, bool directed>
auto BaseGraph<vertex_t, weighted, directed>::isVertex(
    const vertex_t &vertex) const -> bool {

  return m_vertices.contains(vertex);
}

// ################ IS EDGE ##################
template <typename vertex_t, bool weighted, bool directed>
bool BaseGraph<vertex_t, weighted, directed>::isEdge(
    const vertex_t &fromV, const vertex_t &toV) const {
  if (!isVertex(fromV) || !isVertex(toV)) {
    return false;
  }
  return m_adjMatrix[m_vertices.at(fromV)][m_vertices.at(toV)];
}

// ################ ADD VERTEX ##################
template <typename vertex_t, bool weighted, bool directed>
void BaseGraph<vertex_t, weighted, directed>::addVertex(
    const vertex_t &vertex) {
  if (isVertex(vertex)) {
    throw std::runtime_error("Vertex already exists");
  }
  unsigned int Vsize = unsignedCast(m_vertices.size());
  m_vertices[vertex] = Vsize;
  lookup_table[Vsize] = vertex;

  m_adjMatrix.resize(m_vertices.size());
  for (unsigned int i = 0; i < m_vertices.size(); i++) {
    m_adjMatrix[i].resize(m_vertices.size());
  }
}

// ################ VERTEX HAS EDGES ##################
template <typename vertex_t, bool weighted, bool directed>
bool BaseGraph<vertex_t, weighted, directed>::vertexHasEdges(
    const vertex_t &vertex) const {

  if (!isVertex(vertex)) {
    throw std::runtime_error("Vertex does not exist");
  }

  unsigned int index = m_vertices.at(vertex);

  if (std::any_of(m_adjMatrix[index].begin(), m_adjMatrix[index].end(),
                  [](const auto &edge) { return edge; })) {
    return true;
  }
  // check m_adjMatrix[x][index]
  return std::any_of(m_adjMatrix.begin(), m_adjMatrix.end(),
                     [index](const auto &row) { return row[index]; });
  return false;
}

// ################ REMOVE VERTEX ##################
template <typename vertex_t, bool weighted, bool directed>
void BaseGraph<vertex_t, weighted, directed>::removeVertex(
    const vertex_t &vertex, const bool &force) {
  if (!isVertex(vertex)) {
    throw std::runtime_error("Vertex does not exist");
  }
  if (vertexHasEdges(vertex) && !force) {
    throw std::invalid_argument(
        "Vertex has edges.\nMaybe you meant to run with 'force = true'");
    return;
  }

  // remove vertex from m_adjMatrix
  unsigned int index = m_vertices.at(vertex);
  m_adjMatrix.erase(m_adjMatrix.begin() + index);
  for (auto &row : m_adjMatrix) {
    row.erase(row.begin() + index);
  }
  // remove vertex from m_vertices
  m_vertices.erase(vertex);
  lookup_table.erase(index);

  for (const auto &idx : lookup_table) {
    if (idx.first > index) {
      m_vertices[idx.second]--;

      // modifiy key
      auto node = lookup_table.extract(idx.first);
      node.key()--;
      lookup_table.insert(std::move(node));
    }
  }
}

// ################ REMOVE EDGE ##################
template <typename vertex_t, bool weighted, bool directed>
void BaseGraph<vertex_t, weighted, directed>::removeEdge(const vertex_t &fromV,
                                                         const vertex_t &toV) {

  if (!isEdge(fromV, toV)) {
    throw std::runtime_error("Edge does not exist");
  }
  m_adjMatrix[m_vertices.at(fromV)][m_vertices.at(toV)] = 0; // 0 -> false
}

// ################ GET VERTEX COUNT ##################
template <typename vertex_t, bool weighted, bool directed>
auto BaseGraph<vertex_t, weighted, directed>::getVertexCount() const
    -> unsigned int {
  return unsignedCast(m_vertices.size());
}

// ################ GET EDGE COUNT ##################
template <typename vertex_t, bool weighted, bool directed>
auto BaseGraph<vertex_t, weighted, directed>::getEdgeCount() const
    -> unsigned int {
  unsigned int count = 0;
  for (const auto &row : m_adjMatrix) {
    count += std::count(row.begin(), row.end(), true);
  }
  return count;
}

// ################ GET ADJACENT VERTICES ##################
template <typename vertex_t, bool weighted, bool directed>
auto BaseGraph<vertex_t, weighted, directed>::getAdjacentVertices(
    const vertex_t &vertex) const -> std::vector<vertex_t> {

  if (!isVertex(vertex)) {
    throw std::runtime_error("Vertex does not exist");
  }
  vector<vertex_t> adjacentVertices;

  for (const auto &vertex_it : m_vertices) {
    if (m_adjMatrix[m_vertices.at(vertex)][m_vertices.at(vertex_it.first)] ||
        m_adjMatrix[m_vertices.at(vertex_it.first)]
                   [m_vertices.at(vertex)]) { // For directed graphs?
      adjacentVertices.push_back(vertex_it.first);
    }
  }
  return adjacentVertices;
}

// ################ GET VERTICES ##################
template <typename vertex_t, bool weighted, bool directed>
auto BaseGraph<vertex_t, weighted, directed>::getVertices() const
    -> std::vector<vertex_t> {

  vector<vertex_t> verticesList;

  std::transform(m_vertices.begin(), m_vertices.end(),
                 std::back_inserter(verticesList),
                 [](const auto &vertex) { return vertex.first; });
  return verticesList;
}

// ################ GET SHORTEST PATH ##################
template <typename vertex_t, bool weighted, bool directed>
auto BaseGraph<vertex_t, weighted, directed>::getShortestPath(
    const vertex_t &fromV, const vertex_t &toV) const -> std::vector<vertex_t> {

  if (!isVertex(fromV) || !isVertex(toV)) {
    throw std::runtime_error("Vertex does not exist");
  }

  std::queue<vertex_t> Vqueue;

  // vector of visited vertices = false
  std::vector<bool> visited(m_vertices.size(), false);

  // vector of previous vertices
  std::vector<vertex_t> previous(m_vertices.size());

  Vqueue.push(fromV);
  visited[m_vertices.at(fromV)] = true;
  previous[m_vertices.at(fromV)] = fromV;

  vertex_t currentV;

  while (!Vqueue.empty()) {
    currentV = Vqueue.front();
    Vqueue.pop();

    for (const auto &adjV : getAdjacentVertices(currentV)) {
      if (!visited[m_vertices.at(adjV)]) {
        Vqueue.push(adjV);
        visited[m_vertices.at(adjV)] = true;
        previous[m_vertices.at(adjV)] = currentV;
      }
    }
  }

  if (!visited[m_vertices.at(toV)]) {
    throw std::runtime_error("No path exists");
  }

  std::vector<vertex_t> path;
  vertex_t current = toV;

  while (current != fromV) { // floating safe (in theory)
    path.push_back(current);
    current = previous[m_vertices.at(current)];
  }
  path.push_back(fromV);
  return std::vector<vertex_t>(path.rbegin(), path.rend());
}

template <typename vertex_t, bool weighted, bool directed>
auto BaseGraph<vertex_t, weighted, directed>::getAdjacencyMatrix() const
    -> std::vector<std::vector<weight_t>> {
  return m_adjMatrix;
}

// Explicit instantiations
//
// BaseGraph
template class BaseGraph<int, false, false>;
template class BaseGraph<int, true, false>;
template class BaseGraph<std::string, false, false>;
template class BaseGraph<std::string, true, false>;
template class BaseGraph<char, false, false>;
template class BaseGraph<char, true, false>;
template class BaseGraph<float, false, false>;
template class BaseGraph<float, true, false>;
template class BaseGraph<double, false, false>;
template class BaseGraph<double, true, false>;
