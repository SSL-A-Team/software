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

#ifndef DIRECTED_GRAPH_HPP_
#define DIRECTED_GRAPH_HPP_

#include <map>
#include <vector>

/**
 * Directed graph of nodes with no restriction on connectivity
 */
template<typename Node>
class DirectedGraph {
public:
  std::size_t add_node(Node node) {
    std::size_t node_idx = nodes.size();
    nodes.push_back(node);
    root_nodes.push_back(node_idx);
    parent_to_child_relationship[node_idx] = {};

    return node_idx;
  }

  std::size_t add_node(Node node, std::size_t parent_idx) {
    return add_node(node, {parent_idx});
  }

  std::size_t add_node(Node node, std::vector<std::size_t> parent_idxs) {
    std::size_t node_idx = nodes.size();
    nodes.push_back(node);

    for (auto parent_idx : parent_idxs) {
      parent_to_child_relationship[parent_idx].push_back(node_idx);
    }

    return node_idx;
  }

  Node get_node(std::size_t node_idx) const {
    return nodes.at(node_idx);
  }

  std::vector<std::size_t> get_root_nodes() const {
    return root_nodes;
  }

  std::vector<std::size_t> get_children(std::size_t node_idx) const {
    return parent_to_child_relationship.at(node_idx);
  }

private:
  std::vector<Node> nodes;
  std::vector<std::size_t> root_nodes;
  std::map<std::size_t, std::vector<std::size_t>> parent_to_child_relationship;
};

#endif  //DIRECTED_GRAPH_HPP_