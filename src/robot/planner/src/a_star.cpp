#include "a_star.hpp"

AStar::AStar(CellIndex start, CellIndex end, const Grid &grid) : 
    start(start), end(end), grid(grid) {}

std::vector<CellIndex> AStar::run() {
    // Insert start node into open list
    std::shared_ptr<AStarNode> node = std::make_shared<AStarNode>(start);
    pq.push(node);
    open[node->index] = node;
    
    while (!pq.empty()) {
        // Get the most optimal point in the open list
        std::shared_ptr<AStarNode> node = pq.top();
        pq.pop();
        open.erase(node->index);
        close[node->index] = node;
        
        // Check if this point is the end            
        if (node->index == end) {
            // Retrace path based on parent pointers
            return retrace_path(node);
        }
        
        // For each of the 8 neighbours
        for (int i = 0; i < 8; i++) {
            int new_x = node->index.x + directionX[i];
            int new_y = node->index.y + directionY[i];
            
            if (new_x >= 0 && new_y >= 0 && new_x < grid.width && new_y < grid.height) {
                // If neighbour is obstacle or in closed list
                if (grid.data[new_y * grid.width + new_x] == 1 || (close.find({new_x, new_y}) != close.end())) {
                    continue;
                }
                
                // Get new scores for neighbours based on the current path
                double new_g_score = calculate_g_score({new_x, new_y}, node->index, node->g_score);
                double new_h_score = calculate_h_score({new_x, new_y}, end);
                double new_f_score = new_g_score + new_h_score;
                
                // Update the score if this is a better path
                if (open.find({new_x, new_y}) != open.end()) {
                    if (open[{new_x, new_y}]->f_score > new_f_score) {
                        open[{new_x, new_y}]->g_score = new_g_score;
                        open[{new_x, new_y}]->h_score = new_h_score;
                        open[{new_x, new_y}]->f_score = new_f_score;
                        open[{new_x, new_y}]->parent = node;
                    }
                } 
                else {
                    std::shared_ptr<AStarNode> new_node = std::make_shared<AStarNode>(CellIndex{new_x, new_y}, new_g_score, new_h_score, new_f_score, node);
                    pq.push(new_node);
                    open[{new_x, new_y}] = new_node;
                }
            }
        }
    }
    
    return std::vector<CellIndex>{{-1, -1}};
}

double AStar::calculate_g_score(const CellIndex &current, const CellIndex &parent, double parent_g_score) {
    return parent_g_score + diagonal_distance(current, parent);
}

double AStar::calculate_h_score(const CellIndex &current, const CellIndex &end) {
    return diagonal_distance(current, end);
}

double AStar::diagonal_distance(const CellIndex &c1, const CellIndex &c2) {
    int d_max = std::max(std::abs(c1.x - c2.x), std::abs(c1.y - c2.y));
    int d_min = std::min(std::abs(c1.x - c2.x), std::abs(c1.y - c2.y));
    
    return diag_score * d_min + non_diag_score * (d_max - d_min);
}

std::vector<CellIndex> AStar::retrace_path(std::shared_ptr<AStarNode> node) {
    std::vector<CellIndex> path;
    while(node != nullptr) {
        path.push_back(node->index);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}