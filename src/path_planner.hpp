#pragma once
#include <vector>
#include <set>
#include <map>
#include <queue>
#include <cmath>
#include <opencv2/opencv.hpp>

// ==================== Grid Cell Types ====================
enum class CellType {
    UNKNOWN = 0,    // Unexplored
    FREE = 1,       // Explored and free
    OBSTACLE = 2,   // Obstacle detected
    YELLOW_LINE = 3,// Yellow boundary
    ARUCO = 4       // ArUco marker location
};

// ==================== Grid Cell ====================
struct GridCell {
    int x, y;
    CellType type;
    double exploredTime;  // Timestamp when explored
    int visitCount;       // How many times visited
    
    GridCell() : x(0), y(0), type(CellType::UNKNOWN), exploredTime(0.0), visitCount(0) {}
    GridCell(int x_, int y_) : x(x_), y(y_), type(CellType::UNKNOWN), exploredTime(0.0), visitCount(0) {}
    
    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
};

// ==================== Waypoint ====================
struct Waypoint {
    double x, y;  // Position in cm
    int priority; // Higher = more important
    bool reached;
    
    Waypoint() : x(0.0), y(0.0), priority(0), reached(false) {}
    Waypoint(double x_, double y_, int p = 0) : x(x_), y(y_), priority(p), reached(false) {}
};

// ==================== Grid Map ====================
class GridMap {
public:
    GridMap(double widthCm, double heightCm, double resolutionCm);
    
    // Cell operations
    void setCell(double xCm, double yCm, CellType type);
    CellType getCell(double xCm, double yCm) const;
    void markExplored(double xCm, double yCm);
    void incrementVisitCount(double xCm, double yCm);
    
    // Grid conversion
    void worldToGrid(double xCm, double yCm, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& xCm, double& yCm) const;
    bool isInBounds(int gx, int gy) const;
    
    // Query functions
    bool isFree(double xCm, double yCm) const;
    bool isObstacle(double xCm, double yCm) const;
    bool isExplored(double xCm, double yCm) const;
    int getVisitCount(double xCm, double yCm) const;
    double getExplorationRatio() const;  // Fraction of map explored
    
    // Visualization
    cv::Mat toMat() const;  // Convert to OpenCV Mat for visualization
    
    // Getters
    int getWidth() const { return gridWidth_; }
    int getHeight() const { return gridHeight_; }
    double getResolution() const { return resolution_; }
    
private:
    int gridWidth_, gridHeight_;
    double resolution_;  // cm per cell
    double worldWidth_, worldHeight_;
    std::vector<std::vector<GridCell>> grid_;
};

// ==================== A* Node ====================
struct AStarNode {
    int x, y;
    double g;  // Cost from start
    double h;  // Heuristic to goal
    double f;  // Total cost (g + h)
    AStarNode* parent;
    
    AStarNode(int x_, int y_, double g_, double h_, AStarNode* p = nullptr)
        : x(x_), y(y_), g(g_), h(h_), f(g_ + h_), parent(p) {}
    
    bool operator>(const AStarNode& other) const {
        return f > other.f;
    }
};

// ==================== A* Planner ====================
class AStarPlanner {
public:
    AStarPlanner(GridMap* map);
    
    // Plan path from start to goal (in cm coordinates)
    std::vector<Waypoint> planPath(double startX, double startY, 
                                     double goalX, double goalY);
    
    // Check if path is clear
    bool isPathClear(double x1, double y1, double x2, double y2);
    
private:
    GridMap* map_;
    
    // Heuristic functions
    double heuristic(int x1, int y1, int x2, int y2);
    
    // Get neighbors (4-connected or 8-connected)
    std::vector<std::pair<int, int>> getNeighbors(int x, int y, bool use8Connected = true);
    
    // Cost calculation
    double getCost(int x, int y);
};

// ==================== Spiral Explorer ====================
class SpiralExplorer {
public:
    SpiralExplorer(GridMap* map);
    
    // Generate spiral exploration waypoints from center
    std::vector<Waypoint> generateSpiralWaypoints(double centerX, double centerY, 
                                                    double maxRadius, int numWaypoints);
    
    // Get next best exploration target based on current position
    Waypoint getNextExplorationTarget(double currentX, double currentY,
                                       const std::set<int>& visitedAruco);
    
    // Update exploration priority based on ArUco marker history
    void updatePriorities(const std::set<int>& visitedAruco,
                          const std::map<int, std::pair<double, double>>& arucoPositions);
    
private:
    GridMap* map_;
    std::vector<Waypoint> explorationWaypoints_;
    
    // Calculate exploration value for a cell
    double calculateExplorationValue(double x, double y, double currentX, double currentY);
    
    // Generate spiral pattern (Archimedean spiral)
    std::vector<std::pair<double, double>> generateSpiralPattern(double centerX, double centerY,
                                                                   double maxRadius, int numPoints);
};

// ==================== Path Smoother ====================
class PathSmoother {
public:
    // Smooth path to remove unnecessary waypoints
    static std::vector<Waypoint> smoothPath(const std::vector<Waypoint>& path, GridMap* map);
    
    // Simplify path using Douglas-Peucker algorithm
    static std::vector<Waypoint> simplifyPath(const std::vector<Waypoint>& path, double epsilon);
};

