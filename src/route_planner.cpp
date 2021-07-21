#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
	start_node = &m_Model.FindClosestNode(start_x, start_y);
	end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
	for (auto node : current_node->neighbors) {
		node->parent = current_node;
		node->g_value = current_node->g_value + current_node->distance(*node);
		node->h_value = CalculateHValue(node);
		open_list.emplace_back(node);
		node->visited = true;
	}
}

RouteModel::Node *RoutePlanner::NextNode() {
	std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node* a, RouteModel::Node* b) {
		return (a->g_value + a->h_value) < (b->g_value + b->h_value);
	});
	RouteModel::Node* lowest_sum = open_list.front();
	open_list.erase(open_list.begin());
	return lowest_sum;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

	while (current_node != nullptr) {
		path_found.emplace_back(*current_node);
		if (current_node->parent != nullptr) distance += current_node->distance(*(current_node->parent));
		current_node = current_node->parent;
	}
	std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
	start_node->visited = true;
	open_list.emplace_back(start_node);

	while (!open_list.empty()) {
		current_node = NextNode();
		if (current_node->distance(*end_node) == 0) {
			m_Model.path = ConstructFinalPath(current_node);
			return;
		}
		AddNeighbors(current_node);
	}
}
