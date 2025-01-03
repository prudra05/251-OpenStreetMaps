#pragma once

#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

using namespace std;

/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type

template <typename VertexT, typename WeightT>
class graph {
 private:

  set<VertexT> vertices; //set holding unique vertices + weight
  int vertexCount; // number of vertices
  int edgeCount; // number of edges

 public:
  /// Default constructor
  unordered_map<VertexT, unordered_map<VertexT, WeightT>> adjList; //holds the neighbors and weight of vertices

  graph() {
    this->vertexCount = 0;
    this->edgeCount = 0;
  }

  /// @brief Add the vertex `v` to the graph, must typically be O(1).
  /// @param v
  /// @return true if successfully added; false if it existed already
  bool addVertex(VertexT v) {
    // TODO_STUDENT
    if (vertices.find(v) != vertices.end()) { 
      return false; // Vertex already exists 
    } 

    //adjList.emplace_back(); // Add a new list for the new vertex 
    vertices.insert(v); // Add vertex to the set 
    vertexCount++; 
    return true;
  }

  /// @brief Add or overwrite directed edge in the graph, must typically be
  /// O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight edge weight / label
  /// @return true if successfully added or overwritten;
  ///         false if either vertices isn't in graph

  bool addEdge(VertexT from, VertexT to, WeightT weight) { 
    if (vertices.count(from) ==  0 || vertices.count(to) == 0 ) { 
      return false;
    }

    if (adjList[from].count(to) != 1) {
      edgeCount++;
    }

    adjList[from][to] = weight; // Insert the edge with the weight
    return true; 
  }
  

  /// @brief Maybe get the weight associated with a given edge, must typically
  /// be O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight output parameter
  /// @return true if the edge exists, and `weight` is set;
  ///         false if the edge does not exist

  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    auto it = adjList.find(from); 
    if (it != adjList.end()) { 
      auto edgeIt = it->second.find(to);  //access second part of map
      if (edgeIt != it->second.end()) { 
        weight = edgeIt->second; 
        return true; 
      } 
    } 
    return false;
  }

    WeightT getWeightValue(VertexT from, VertexT to) const {
      auto it = adjList.find(from); 
      if (it != adjList.end()) { 
        auto edgeIt = it->second.find(to);  //access second part of map
        if (edgeIt != it->second.end()) { 
          return edgeIt->second;
        } 
      } 
      return 0;
    }

  /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
  /// @param v
  /// @return vertices that v has an edge to

  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;
    // TODO_STUDENT
    auto it = adjList.find(v); //finds vertice v within adjList
    if (it != adjList.end()) { // if it is in adjList
      for (const auto& edge : it->second) { //accesses all the neighbors of v
        S.insert(edge.first); //stores all edges in S
      }
    }
    
    return S;
  }

  /// @brief Return a vector containing all vertices in the graph
  vector<VertexT> getVertices() const {
    // TODO_STUDENT
    vector<VertexT> v;
    for (auto it = vertices.begin(); it != vertices.end(); ++it) {
      v.push_back(*it);
    }
    return v;
  }

  /// @brief Get the number of vertices in the graph. Runs in O(1).
  size_t numVertices() const {
    // TODO_STUDENT
    return vertexCount;
  }

  /// @brief Get the number of directed edges in the graph. Runs in at most
  /// O(|V|), but should be O(1).
  size_t numEdges() const {
    // TODO_STUDENT
    return edgeCount;
  }
};
