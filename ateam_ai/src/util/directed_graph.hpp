// Copyright 2021 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef UTIL__DIRECTED_GRAPH_HPP_
#define UTIL__DIRECTED_GRAPH_HPP_

#include <ateam_common/status.hpp>

#include <unordered_map>
#include <vector>

template<typename Node>
class DirectedGraph;

/**
 * Directed graph of nodes. The graph must not be looping
 */
template<typename Node>
class DirectedGraph
{
public:
  std::size_t add_node(Node node)
  {
    std::size_t node_idx = nodes.size();
    nodes.push_back(node);
    root_nodes.push_back(node_idx);
    parent_to_child_relationship[node_idx] = {};

    return node_idx;
  }

  std::size_t add_node(Node node, std::size_t parent_idx)
  {
    return add_node(node, std::vector<std::size_t>{parent_idx});
  }

  std::size_t add_node(Node node, std::vector<std::size_t> parent_idxs)
  {
    std::size_t child_idx = nodes.size();
    nodes.push_back(node);
    parent_to_child_relationship[child_idx] = {};

    for (auto parent_idx : parent_idxs) {
      ATEAM_CHECK(parent_to_child_relationship.count(parent_idx) > 0, "Given parent ID does not exist in parent->child relationship");
      parent_to_child_relationship.at(parent_idx).push_back(child_idx);
    }

    return child_idx;
  }

  Node get_node(std::size_t node_idx) const
  {
    ATEAM_CHECK(node_idx < nodes.size(), "node_idx must be smaller than the number of nodes");
    return nodes.at(node_idx);
  }

  std::vector<std::size_t> get_root_nodes() const
  {
    return root_nodes;
  }

  std::vector<std::size_t> get_children(std::size_t node_idx) const
  {
    return parent_to_child_relationship.at(node_idx);
  }

  // Self friend because template classes of different types are "difference classes"
  // so we need to friend ourself so we can access the private members of all
  // templated versions of the DirectedGraph<T> from DirectedGraph<Node>
  template<typename T>
  friend class DirectedGraph;

  template<typename T>
  DirectedGraph<T> copy_shape_with_new_type(std::map<std::size_t, T> new_nodes) const {
    DirectedGraph<T> new_graph;
    new_graph.root_nodes = root_nodes;
    new_graph.parent_to_child_relationship = parent_to_child_relationship;
    for (std::size_t i = 0; i < nodes.size(); i++) {
      if (new_nodes.count(i) > 0) {
        new_graph.nodes.push_back(new_nodes.at(i));
      } else {
        new_graph.nodes.push_back(T());
      }
    }

    return new_graph;
  }

private:
  std::vector<Node> nodes;
  std::vector<std::size_t> root_nodes;
  std::unordered_map<std::size_t, std::vector<std::size_t>> parent_to_child_relationship;
};


#endif  // UTIL__DIRECTED_GRAPH_HPP_
