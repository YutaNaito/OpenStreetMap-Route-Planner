#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Using the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &(m_Model.FindClosestNode(start_x, start_y));
    this->end_node = &(m_Model.FindClosestNode(end_x, end_y));
}


/**
 * Compare the f_value of two nodes
 * 
 * @param Pointer to the current Node to calculate
 * 
 * @return H_value
*/
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  // determine the distance to the end node
  return node->distance(*this->end_node);
}


/**
 * Expand the current node by adding all unvisited neighbors to the open list.
 * 
 * @param Pointer to the current Node to add neighbors node 
*/
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  // populate current_node.neighbors vector with all the neighbors.
  current_node->FindNeighbors();

  // set the parent, the h_value, the g_value
  // add the neighbor to open_list and set the node's visited attribute to true
  for (auto *neighbor : current_node->neighbors){
    neighbor->visited = true;
    neighbor->parent = current_node;
    neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
    neighbor->h_value = CalculateHValue(neighbor);
    this->open_list.emplace_back(neighbor);
  }
}

/**
 * Compare the f_value of two nodes
 * 
 * @param first  Pointer to the first  Node to compare
 * @param second Pointer to the second Node to compare
 * 
 * @return True if the first f_value is less than second f_value
*/
bool RoutePlanner::Compare(const RouteModel::Node *first, const RouteModel::Node *second) {
    // f_value is determined by adding g_value to h_value
    return (first->g_value + first->h_value) < (second->g_value + second->h_value);
}


/**
 * Sorts the open list
 * 
 * @return True if the first f_value is less than second f_value
*/
RouteModel::Node *RoutePlanner::NextNode() {

  // Sort the open_list according to the sum of the h value and g value.
  std::sort(open_list.begin(), open_list.end(), Compare);

  // Temporary variable to hold the position of the nearest node
  auto *next_node = open_list.front();

  // Remove node from open nodes to avoid revisiting it
  open_list.erase(open_list.begin());
  
  return next_node;
}


/**
 * Construct the final path found from your A* search
 * 
 * @param Pointer to the current Node
 * 
 * @return final path
*/
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
  
    // iteratively follow the chain of parents of nodes until the starting node is found.
    while(this->start_node->x != current_node->x && this->start_node->y != current_node->y){
      // add the distance from the node to its parent to the distance variable
      distance += current_node->distance(*(current_node->parent));
      path_found.push_back(*current_node);
      current_node = current_node->parent;
    }
  
    // path_found should be in the correct order: 
    // - the start node should be the first element of the vector
    // - the end node should be the last element.
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());
  
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


/**
 * A* search
*/
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    this->start_node->visited = true;
    current_node = this->start_node;
    this->open_list.push_back(current_node);
  
    while(this->open_list.size() > 0 && current_node != this->end_node){
      // add all of the neighbors of the current node to the open_list
      AddNeighbors(current_node);
      
      // sort the open_list and return the next node
      current_node = NextNode();
    }
  
    // Store the final path in the m_Model.path attribute before the method exits
    // that path will be displayed on the map tile
    m_Model.path = ConstructFinalPath(end_node);
}