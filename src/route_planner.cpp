#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return  (*node).distance(*end_node);  
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  
           std::vector<RouteModel::Node*>:: iterator iter;
           for (iter = current_node->neighbors.begin(); iter != current_node->neighbors.end(); ++iter) {
                  (*iter)->parent = current_node;
                  (*iter)->h_value = CalculateHValue(*iter);
                  (*iter)->visited = true;
                  (*iter)->g_value = current_node->g_value + current_node->distance(*(*iter));
                  open_list.push_back(*iter);
            }  
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool comp_func(const RouteModel::Node* pn_a, const RouteModel::Node* pn_b) {
    float g_a = pn_a->g_value;
    float h_a = pn_a->h_value;
    
    float g_b = pn_b->g_value;
    float h_b = pn_b->h_value;
    
    return ((g_a + h_a) > (g_b + h_b)); //> creates descending vector    
    
  }

RouteModel::Node *RoutePlanner::NextNode() {

  RouteModel::Node* LowestSumNode = nullptr;
  std::vector<int> g_h_vec;
  
  std::sort(open_list.begin(), open_list.end(), comp_func);
  LowestSumNode = open_list.back();
  
  open_list.erase(open_list.end() - 1);  
  
  return LowestSumNode;  
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node next_node;
  
    // TODO: Implement your solution here.  
    bool start_found = false;
    
  while (!start_found) {  
        
    if (path_found.empty()){
      next_node = *current_node;
    }
    else {  
      RouteModel::Node front_node = path_found.front();
      next_node = *front_node.parent;  
      
      distance += next_node.distance(path_found.front()); 

      }
    
    path_found.insert(path_found.begin(), next_node);
   
    start_found = ( (start_node->x == next_node.x) && (start_node->y == next_node.y) );
  } 
   
  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
  bool at_end_node = false;
  std::vector<RouteModel::Node> final_path;
  
  RouteModel::Node next_node;
  
  current_node = start_node; //pointer
  

  while (!at_end_node) {
    
    AddNeighbors(current_node);
    
    current_node = NextNode();
    
    at_end_node = (current_node == end_node);   
    
  }  
 
  final_path = ConstructFinalPath(current_node);
  
  m_Model.path = final_path;

}