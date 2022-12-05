#include "Graph.h"

using std::vector;

/*
 ###################################################################
 ################# UNWEIGHTED GRAPH IMPLEMENTATION #################
 ###################################################################
*/

template <typename vertex_t>
auto Graph<vertex_t, false>::getEdgeList()
    -> vector<std::pair<vertex_t, vertex_t>> {

  vector<std::pair<vertex_t, vertex_t>> edgeList;

  /*
  for (auto const &var : vertices) {
    for (auto const &urn : vertices) {
      if (isEdge(var, urn)) {
        edgeList.push_back(std::make_pair(var, urn));
      }
    }
  } // O(v²) + O(v²)? = O(v²) fuck it
  */

  for (unsigned int rowIdx = 0; const auto &row : m_adjMatrix) {
    for (unsigned int colIdx = 0; const auto &col : row) {
      if (col) {
        edgeList.emplace_back(
            std::make_pair(lookup_table.at(rowIdx), lookup_table.at(colIdx)));
      }
    }
  } // O(V²)

  return edgeList;
}

template <typename vertex_t>
void Graph<vertex_t, false>::addEdge(const vertex_t &fromV,
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

// explicit instantiation
template class Graph<int, false>;
template class Graph<int, true>;
template class Graph<std::string, false>;
