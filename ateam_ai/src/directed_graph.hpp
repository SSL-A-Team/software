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
  std::map<std::vector<std::size_t>> parent_to_child_relationship;
};