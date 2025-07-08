#include "node.h"
#include <sstream>
#include <algorithm>

namespace heuristic_search {

Node::Node(const std::string& state, CostType g_cost, CostType h_cost)
    : state_(state), g_cost_(g_cost), h_cost_(h_cost), parent_(nullptr), action_("") {
}

std::vector<std::string> Node::getPath() const {
    std::vector<std::string> path;
    const Node* current = this;
    
    // Reconstruct path by following parent pointers
    while (current->parent_ != nullptr) {
        if (!current->action_.empty()) {
            path.push_back(current->action_);
        }
        current = current->parent_.get();
    }
    
    // Reverse to get correct order (start to goal)
    std::reverse(path.begin(), path.end());
    return path;
}

bool Node::equals(const Node& other) const {
    return state_ == other.state_;
}

std::string Node::toString() const {
    std::ostringstream oss;
    oss << "Node{state='" << state_ 
        << "', g=" << g_cost_ 
        << ", h=" << h_cost_ 
        << ", f=" << getFCost() 
        << ", action='" << action_ << "'}";
    return oss.str();
}

Node::NodePtr Node::clone() const {
    auto new_node = std::make_shared<Node>(state_, g_cost_, h_cost_);
    new_node->parent_ = parent_;
    new_node->action_ = action_;
    return new_node;
}

} // namespace heuristic_search 