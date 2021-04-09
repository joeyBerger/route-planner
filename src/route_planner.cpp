#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto node : current_node->neighbors) {
        if (node->visited) continue;
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + node->distance(*current_node);
        node->visited = true;
        open_list.push_back(node);
    }
}


bool RoutePlanner::Compare(RouteModel::Node *a, RouteModel::Node *b) {
    float dist0 = a->h_value + a->g_value;
    float dist1 = b->h_value + b->g_value;
    return dist0 < dist1;
}


RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(),Compare);
    auto lowest_node = open_list[0];
    open_list.erase(open_list.begin(),open_list.begin()+1);
    return lowest_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.push_back(*current_node);
    while(true) {
        auto node = path_found[path_found.size()-1];
        path_found.push_back(*node.parent);
        distance += node.distance(*node.parent);
        if (node.parent == start_node) break;
    }

    std::reverse(path_found.begin(),path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    start_node->visited = true;
    current_node = start_node;

    AddNeighbors(current_node);
    int counter = 0;
    while (open_list.size() > 0) {
        current_node = NextNode();
        if (current_node->x == end_node->x && current_node->y == end_node->y) {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        AddNeighbors(current_node);
    }
    if (open_list.size() == 0) std::cout << "Could not construct path.\n";
}