#include "Graph.h"
#include <algorithm>
#include <array>
#include <limits>
#include <queue>

using std::vector;

/*
 ###################################################################
 ################# UNWEIGHTED GRAPH IMPLEMENTATION #################
 ###################################################################
*/

template <typename vertex_t, bool directed>
auto Graph<vertex_t, false, directed>::getEdgeList() const
    -> vector<std::pair<vertex_t, vertex_t>> {

  vector<std::pair<vertex_t, vertex_t>> edgeList;

  /*
  for (auto const &var : vertices) {
    for (auto const &urn : vertices) {
      if (isEdge(var, urn)) {
        edgeList.push_back(std::make_pair(var, urn));
      }
    }
  } // O(v²) * O(v²)? [isEdge] = O(v²) fuck it
  */

  for (unsigned int rowIdx = 0; const auto &row : m_adjMatrix) {
    for (unsigned int colIdx = 0; const auto &col : row) {
      if (col) {
        edgeList.emplace_back(
            std::make_pair(lookup_table.at(rowIdx), lookup_table.at(colIdx)));
      }
      colIdx++;
    }
    rowIdx++;
  } // O(V²)

  return edgeList;
}

template <typename vertex_t, bool directed>
void Graph<vertex_t, false, directed>::addEdge(const vertex_t &fromV,
                                               const vertex_t &toV) {
  try {
    addVertex(fromV);
  } catch (...) { // No problem if it already exists
  }
  try {
    addVertex(toV);
  } catch (...) { // No problem if it already exists
  }

  if (m_adjMatrix[m_vertices.at(fromV)][m_vertices.at(toV)]) {
    throw std::runtime_error("Edge already exists");
  }

  m_adjMatrix[m_vertices.at(fromV)][m_vertices.at(toV)] = true;
  m_adjMatrix[m_vertices.at(toV)][m_vertices.at(fromV)] = true;
}

/*
 ###################################################################
 ################# WEIGHTED GRAPH IMPLEMENTATION #################
 ###################################################################
*/

template <typename vertex_t, bool directed>
auto Graph<vertex_t, true, directed>::getEdgeList() const
    -> vector<std::tuple<vertex_t, vertex_t, unsigned int>> {

  vector<std::tuple<vertex_t, vertex_t, unsigned int>> edgeList;

  for (unsigned int rowIdx = 0; const auto &row : m_adjMatrix) {
    for (unsigned int colIdx = 0; const unsigned int &col : row) {

      // find inverse in edgeList
      bool inverseFound = std::find(edgeList.begin(), edgeList.end(),
                                    std::make_tuple(lookup_table.at(colIdx),
                                                    lookup_table.at(rowIdx),
                                                    col)) != edgeList.end();

      if (col > 0 && !inverseFound) {
        edgeList.emplace_back(std::make_tuple(lookup_table.at(rowIdx),
                                              lookup_table.at(colIdx), col));
      }
      colIdx++;
    }
    rowIdx++;
  } // O(V²)

  return edgeList;
}

template <typename vertex_t, bool directed>
void Graph<vertex_t, true, directed>::addEdge(const vertex_t &fromV,
                                              const vertex_t &toV,
                                              const unsigned int &weight) {
  try {
    addVertex(fromV);
  } catch (...) { // No problem if it already exists
  }
  try {
    addVertex(toV);
  } catch (...) { // No problem if it already exists
  }

  if (m_adjMatrix[m_vertices.at(fromV)][m_vertices.at(toV)]) {
    throw std::runtime_error("Edge already exists");
  }

  if (weight == 0) {
    throw std::runtime_error("Weight cannot be zero");
  }

  m_adjMatrix[m_vertices.at(fromV)][m_vertices.at(toV)] = weight;
  m_adjMatrix[m_vertices.at(toV)][m_vertices.at(fromV)] = weight;
}

template <typename vertex_t, bool directed>
auto Graph<vertex_t, true, directed>::getCheapestPath(const vertex_t &fromV,
                                                      const vertex_t &toV) const
    -> std::vector<std::pair<unsigned int, vertex_t>> {

  if (!isVertex(fromV) || !isVertex(toV)) {
    throw std::runtime_error("Vertex does not exist");
  }

  // vector of visited vertices = false
  std::vector<bool> visited(m_vertices.size(), false);

  // vector of previous vertices
  std::vector<std::pair<unsigned int, vertex_t>> previous(m_vertices.size());

  // vector of distances | costs
  std::vector<unsigned int> distances(m_vertices.size(),
                                      std::numeric_limits<unsigned int>::max());

  visited[m_vertices.at(fromV)] = true;
  previous[m_vertices.at(fromV)] = std::make_pair(0, fromV);
  distances[m_vertices.at(fromV)] = 0;

  // dijkstra's algorithm for adjacency matrix

  std::priority_queue<std::pair<unsigned int, vertex_t>,
                      std::vector<std::pair<unsigned int, vertex_t>>,
                      std::greater<>>
      queue;

  queue.emplace(std::make_pair(0, fromV));

  while (!queue.empty()) {
    auto current = queue.top();
    queue.pop();

    visited[m_vertices.at(current.second)] = true;

    for (const auto &vertex : m_vertices) {

      if (!(m_adjMatrix[m_vertices.at(current.second)][vertex.second] > 0) ||
          visited[vertex.second]) {
        continue;
      }

      auto new_distance =
          distances[m_vertices.at(current.second)] +
          m_adjMatrix[m_vertices.at(current.second)][vertex.second];

      if (new_distance >= distances[vertex.second]) {
        continue;
      }
      distances[vertex.second] = new_distance;
      previous[vertex.second] = std::make_pair(new_distance, current.second);
      queue.emplace(std::make_pair(new_distance, vertex.first));
    }
  }

  if (!visited[m_vertices.at(toV)]) {
    throw std::runtime_error("No path exists");
  }

  // generate path from previous
  std::vector<std::pair<unsigned int, vertex_t>> path;

  auto current = toV;

  while (current != fromV) {
    path.push_back(
        std::make_pair(previous[m_vertices.at(current)].first, current));
    current = previous[m_vertices.at(current)].second;
  }
  path.push_back(std::make_pair(0, fromV));

  return std::vector<std::pair<unsigned int, vertex_t>>(path.rbegin(),
                                                        path.rend());
}

/*
template <typename vertex_t, bool directed>
template <OutputType Type>
auto Graph<vertex_t, true, directed>::formatGraphOutput(
    const WeightedEdgeList &edgeList) const
    -> triConditional_t<Type, vertex_t, true> {
  return edgeList;
}
*/

/*
template <typename vertex_t, bool directed>
template <OutputType Type>
auto Graph<vertex_t, true, directed>::getMinimumExpansionTree() const
    -> triConditional_t<Type, vertex_t, true> {

  auto vertices = getVertices();

  auto edgeList = getEdgeList();

  // sort edges by weight
  std::sort(edgeList.begin(), edgeList.end(), [](auto &left, auto &right) {
    return std::get<2>(left) < std::get<2>(right);
  });

  std::vector<std::tuple<vertex_t, vertex_t, unsigned int>> met;

  for (const auto &edge : edgeList) {

    if (vertices.empty()) {
      break;
    }

    auto fromV =
        std::find_if(vertices.begin(), vertices.end(), [edge](auto vertex) {
          return vertex == std::get<0>(edge);
        });
    auto toV =
        std::find_if(vertices.begin(), vertices.end(), [edge](auto vertex) {
          return vertex == std::get<1>(edge);
        });

    if (fromV == vertices.end() && toV == vertices.end()) {
      continue;
    }
    vertices.erase(fromV);
    vertices.erase(toV);

    met.push_back(edge);
  }

  triConditional_t<Type, vertex_t, true> return_value;

  switch (Type) {
  case OutputType::EdgeList:
    return_value = formatGraph<Type>(met);
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
}
*/

// explicit instantiation
template class Graph<int, false, false>;
template class Graph<int, true, false>;
template class Graph<std::string, false, false>;
template class Graph<std::string, true, false>;
template class Graph<char, false, false>;
template class Graph<char, true, false>;
template class Graph<float, false, false>;
template class Graph<float, true, false>;
template class Graph<double, false, false>;
template class Graph<double, true, false>;
