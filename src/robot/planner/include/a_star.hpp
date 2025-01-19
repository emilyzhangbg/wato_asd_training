#ifndef A_STAR_HPP
#define A_STAR_HPP

#include <iostream>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>

struct Grid {
    int width;
    int height;
    int8_t* data;

    Grid(int w, int h, int8_t* data_ptr)
        : width(w), height(h), data(data_ptr) {}
};

struct CellIndex {
    int x;
    int y;
    
    CellIndex(int x, int y) : x(x), y(y) {}
    CellIndex() : x(0), y(0) {}
    
    bool operator==(const CellIndex &other) const {
        return (x == other.x && y == other.y);
    }
    
    bool operator!=(const CellIndex &other) const {
        return (x != other.x || y != other.y);
    }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const {
        // A simple hash combining x and y
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

// Structure representing a node in the A* open set
struct AStarNode {
    CellIndex index;
    double g_score; // distance from start
    double h_score; // distance to end
    double f_score; // f = g + h
    std::shared_ptr<AStarNode> parent{ nullptr };
    
    AStarNode(CellIndex index)
        : index(index), g_score(0), h_score(0), f_score(0) {}
    AStarNode(CellIndex index, double g_score, double h_score, double f_score, std::shared_ptr<AStarNode> parent)
        : index(index), g_score(g_score), h_score(h_score), f_score(f_score), parent(parent) {}
    
    bool operator>(const AStarNode &other) const {
        return f_score > other.f_score;
    }
    bool operator==(const AStarNode &other) const {
        return f_score == other.f_score;
    }
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
    bool operator()(const std::shared_ptr<AStarNode> a, const std::shared_ptr<AStarNode> b)
    {
        // We want the node with the smallest f_score on top
        return a->f_score > b->f_score;
    }
};

// Direction of the 8 neighbours that need to be checked
static const int directionX[] = {-1,  0,  1, -1, 1, -1, 0, 1};
static const int directionY[] = {-1, -1, -1,  0, 0,  1, 1, 1};

class AStar {
public:
    AStar(CellIndex start, CellIndex end, const Grid &grid);
    std::vector<CellIndex> run();

private:
    std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareF> pq;
    std::unordered_map<CellIndex, std::shared_ptr<AStarNode>, CellIndexHash> open;
    std::unordered_map<CellIndex, std::shared_ptr<AStarNode>, CellIndexHash> close;
    
    CellIndex start;
    CellIndex end;
    const Grid &grid;
    
    double diag_score = 1.4;
    double non_diag_score = 1;
    
    // Helpers
    double calculate_g_score(const CellIndex &current, const CellIndex &parent, double parent_g_score);
    double calculate_h_score(const CellIndex &current, const CellIndex &end);
    double diagonal_distance(const CellIndex &c1, const CellIndex &c2);
    std::vector<CellIndex> retrace_path(std::shared_ptr<AStarNode> node);
};

#endif
