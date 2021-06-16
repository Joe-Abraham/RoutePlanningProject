#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    RoutePlanner::start_node = &m_Model.FindClosestNode(start_x, start_y);
    RoutePlanner::end_node = &m_Model.FindClosestNode(end_x, end_y);

}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
    return node->distance(*RoutePlanner::end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) 
{
    current_node->FindNeighbors();
    std::vector<RouteModel::Node *> neighbour = current_node->neighbors;

    for (auto neigh : neighbour)
    {
        neigh->parent = current_node;
        neigh->h_value = CalculateHValue(neigh);
        neigh->g_value = current_node->g_value + current_node->distance(*neigh);
        neigh->visited = true;
        open_list.push_back(neigh);
    }
}


bool Compare(const RouteModel::Node * node1, const RouteModel::Node * node2)
{
  const float f1 = node1->g_value + node1->h_value;
  const float f2 = node2->g_value + node2->h_value;
  
  if (f1 > f2)
  {
    return true;
  }
  else if (f1==f2)
  {
    return node1->h_value > node2->h_value;
  }
  else
  {
    return false;
  }
}

RouteModel::Node *RoutePlanner::NextNode() 
{
    std::sort(open_list.begin(), open_list.end(),Compare);
    RouteModel::Node *last_node = open_list.back();
    open_list.pop_back();
    return last_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node->parent != nullptr) 
    {
        path_found.emplace_back(*current_node);
        const RouteModel::Node parent = *(current_node->parent);
        distance += current_node->distance(parent);
        current_node = current_node->parent;
    }

    path_found.emplace_back(*current_node);
	std::reverse(path_found.begin(), path_found.end());
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
    start_node->visited = true;
    open_list.emplace_back(start_node);

    while(!open_list.empty()) {
        RouteModel::Node *current_node = NextNode();
        if(current_node->distance(*end_node) == 0)
        {
            m_Model.path = ConstructFinalPath(end_node);
            return;
        }
        AddNeighbors(current_node);
    }
}