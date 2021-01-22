#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // change (x, y) [image] to (node) [map]
    RoutePlanner::start_node = &m_Model.FindClosestNode(start_x, start_y);
    RoutePlanner::end_node = &m_Model.FindClosestNode(end_x, end_y);

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // H value = distance to end node 
    const float h_value = node->distance((*RoutePlanner::end_node));
    return h_value;
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Expand the current node by adding all unvisited neighbors to the open list.
    current_node->FindNeighbors();
    // for each neighbors : h, g, parent and visited set
    for (RouteModel::Node* neighb_node : (*current_node).neighbors) {
        neighb_node->parent = current_node;
        neighb_node->h_value = RoutePlanner::CalculateHValue(neighb_node);
        neighb_node->g_value = neighb_node->distance((*current_node)) + current_node->g_value;
        RoutePlanner::open_list.emplace_back(neighb_node);
        neighb_node->visited = true;
    }
}



RouteModel::Node *RoutePlanner::NextNode() {
    //Sort the open list and return the next node.
    // sort open list by f value = g value + h value 
    std::sort(RoutePlanner::open_list.begin(), RoutePlanner::open_list.end(), [](RouteModel::Node * nodeA, RouteModel::Node * nodeB){
        const float fA = nodeA->g_value + nodeA->h_value;
        const float fB = nodeB->g_value + nodeB->h_value;
        return fA > fB; 
    }); 
    // points to lowest = best choice of path
    RouteModel::Node * plowest = nullptr; 
    plowest = RoutePlanner::open_list.back(); 
    RoutePlanner::open_list.pop_back();
    return plowest; 
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node * cnode = current_node; 
    // Take the last node and recover parents
    while (cnode->x != RoutePlanner::start_node->x &&cnode->y != RoutePlanner::start_node->y ){
        distance += cnode->distance(*(cnode->parent));
        // add current node to the begining of the path 
        path_found.insert(path_found.begin(), (*cnode));
        // define new current node : actual parent's node 
        cnode = cnode->parent; 
    }
    path_found.insert(path_found.begin(), (*RoutePlanner::start_node)); // add the starting node at the begining 
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;


    // init starting node 
    current_node = this->start_node; 
    this->start_node->visited = true; 
    this->AddNeighbors(current_node); // find neighboors of the start node 
    
    // case where start node equal end node 
    if (this->start_node->x == this->end_node->x && this->start_node->y == this->end_node->y){
        std::cout << "No valid path found : Starting Point and Ending Point are too close." << "\n"; 
        m_Model.path = {(*this->start_node),(*this->end_node)};
        return;
    }

    // if node available ask for the next node to choose until we reach end node 
    while(this->open_list.size() > 0) {
        current_node = NextNode(); 
        // check if end node reached 
        if (current_node-> x == this->end_node->x && current_node->y == this->end_node->y) {
            m_Model.path = RoutePlanner::ConstructFinalPath(current_node);
            return;
        }
        this->AddNeighbors(current_node);
    }

    std::cout << "No valid path found." << "\n"; 
    m_Model.path = {(*this->start_node)}; 
    return;

}