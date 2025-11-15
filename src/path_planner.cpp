#include "path_planner.hpp"
#include <algorithm>
#include <ctime>
#include <limits>
#include <iostream>

// ==================== GridMap Implementation ====================

GridMap::GridMap(double widthCm, double heightCm, double resolutionCm)
    : resolution_(resolutionCm), worldWidth_(widthCm), worldHeight_(heightCm)
{
    gridWidth_ = static_cast<int>(std::ceil(widthCm / resolutionCm));
    gridHeight_ = static_cast<int>(std::ceil(heightCm / resolutionCm));
    
    // Initialize grid with unknown cells
    grid_.resize(gridHeight_);
    for (int y = 0; y < gridHeight_; y++) {
        grid_[y].resize(gridWidth_);
        for (int x = 0; x < gridWidth_; x++) {
            grid_[y][x] = GridCell(x, y);
        }
    }
}

void GridMap::worldToGrid(double xCm, double yCm, int& gx, int& gy) const
{
    // Origin at center of grid
    double offsetX = worldWidth_ / 2.0;
    double offsetY = worldHeight_ / 2.0;
    
    gx = static_cast<int>(std::floor((xCm + offsetX) / resolution_));
    gy = static_cast<int>(std::floor((yCm + offsetY) / resolution_));
}

void GridMap::gridToWorld(int gx, int gy, double& xCm, double& yCm) const
{
    double offsetX = worldWidth_ / 2.0;
    double offsetY = worldHeight_ / 2.0;
    
    xCm = (gx * resolution_ + resolution_ / 2.0) - offsetX;
    yCm = (gy * resolution_ + resolution_ / 2.0) - offsetY;
}

bool GridMap::isInBounds(int gx, int gy) const
{
    return gx >= 0 && gx < gridWidth_ && gy >= 0 && gy < gridHeight_;
}

void GridMap::setCell(double xCm, double yCm, CellType type)
{
    int gx, gy;
    worldToGrid(xCm, yCm, gx, gy);
    if (isInBounds(gx, gy)) {
        grid_[gy][gx].type = type;
    }
}

CellType GridMap::getCell(double xCm, double yCm) const
{
    int gx, gy;
    worldToGrid(xCm, yCm, gx, gy);
    if (isInBounds(gx, gy)) {
        return grid_[gy][gx].type;
    }
    return CellType::UNKNOWN;
}

void GridMap::markExplored(double xCm, double yCm)
{
    int gx, gy;
    worldToGrid(xCm, yCm, gx, gy);
    if (isInBounds(gx, gy)) {
        if (grid_[gy][gx].type == CellType::UNKNOWN) {
            grid_[gy][gx].type = CellType::FREE;
        }
        grid_[gy][gx].exploredTime = static_cast<double>(time(nullptr));
    }
}

void GridMap::incrementVisitCount(double xCm, double yCm)
{
    int gx, gy;
    worldToGrid(xCm, yCm, gx, gy);
    if (isInBounds(gx, gy)) {
        grid_[gy][gx].visitCount++;
    }
}

bool GridMap::isFree(double xCm, double yCm) const
{
    CellType type = getCell(xCm, yCm);
    return type == CellType::FREE || type == CellType::UNKNOWN;
}

bool GridMap::isObstacle(double xCm, double yCm) const
{
    CellType type = getCell(xCm, yCm);
    return type == CellType::OBSTACLE || type == CellType::YELLOW_LINE;
}

bool GridMap::isExplored(double xCm, double yCm) const
{
    CellType type = getCell(xCm, yCm);
    return type != CellType::UNKNOWN;
}

int GridMap::getVisitCount(double xCm, double yCm) const
{
    int gx, gy;
    worldToGrid(xCm, yCm, gx, gy);
    if (isInBounds(gx, gy)) {
        return grid_[gy][gx].visitCount;
    }
    return 0;
}

double GridMap::getExplorationRatio() const
{
    int explored = 0;
    int total = gridWidth_ * gridHeight_;
    
    for (int y = 0; y < gridHeight_; y++) {
        for (int x = 0; x < gridWidth_; x++) {
            if (grid_[y][x].type != CellType::UNKNOWN) {
                explored++;
            }
        }
    }
    
    return static_cast<double>(explored) / static_cast<double>(total);
}

cv::Mat GridMap::toMat() const
{
    cv::Mat mat(gridHeight_, gridWidth_, CV_8UC3);
    
    for (int y = 0; y < gridHeight_; y++) {
        for (int x = 0; x < gridWidth_; x++) {
            cv::Vec3b color;
            switch (grid_[y][x].type) {
                case CellType::UNKNOWN:
                    color = cv::Vec3b(50, 50, 50);  // Dark gray
                    break;
                case CellType::FREE:
                    color = cv::Vec3b(200, 200, 200);  // Light gray
                    break;
                case CellType::OBSTACLE:
                    color = cv::Vec3b(0, 0, 255);  // Red
                    break;
                case CellType::YELLOW_LINE:
                    color = cv::Vec3b(0, 255, 255);  // Yellow
                    break;
                case CellType::ARUCO:
                    color = cv::Vec3b(0, 255, 0);  // Green
                    break;
            }
            mat.at<cv::Vec3b>(y, x) = color;
        }
    }
    
    return mat;
}

// ==================== A* Planner Implementation ====================

AStarPlanner::AStarPlanner(GridMap* map) : map_(map) {}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2)
{
    // Euclidean distance
    int dx = x2 - x1;
    int dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<std::pair<int, int>> AStarPlanner::getNeighbors(int x, int y, bool use8Connected)
{
    std::vector<std::pair<int, int>> neighbors;
    
    // 4-connected
    const int dx4[] = {0, 1, 0, -1};
    const int dy4[] = {-1, 0, 1, 0};
    
    // 8-connected (adds diagonals)
    const int dx8[] = {0, 1, 1, 1, 0, -1, -1, -1};
    const int dy8[] = {-1, -1, 0, 1, 1, 1, 0, -1};
    
    int numDirs = use8Connected ? 8 : 4;
    const int* dx = use8Connected ? dx8 : dx4;
    const int* dy = use8Connected ? dy8 : dy4;
    
    for (int i = 0; i < numDirs; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (map_->isInBounds(nx, ny)) {
            neighbors.push_back({nx, ny});
        }
    }
    
    return neighbors;
}

double AStarPlanner::getCost(int x, int y)
{
    double worldX, worldY;
    map_->gridToWorld(x, y, worldX, worldY);
    
    CellType type = map_->getCell(worldX, worldY);
    
    double baseCost = 1.0;
    
    switch (type) {
        case CellType::OBSTACLE:
        case CellType::YELLOW_LINE:
            return 1000.0;  // Very high cost (practically impassable)
        case CellType::UNKNOWN:
            return baseCost * 2.0;  // Prefer explored areas
        case CellType::FREE:
            return baseCost;
        case CellType::ARUCO:
            return baseCost * 1.5;  // Slight penalty to avoid revisiting
    }
    
    return baseCost;
}

std::vector<Waypoint> AStarPlanner::planPath(double startX, double startY, 
                                               double goalX, double goalY)
{
    std::vector<Waypoint> path;
    
    // Convert to grid coordinates
    int startGX, startGY, goalGX, goalGY;
    map_->worldToGrid(startX, startY, startGX, startGY);
    map_->worldToGrid(goalX, goalY, goalGX, goalGY);
    
    if (!map_->isInBounds(startGX, startGY) || !map_->isInBounds(goalGX, goalGY)) {
        return path;  // Invalid start or goal
    }
    
    // Priority queue for open set
    auto cmp = [](AStarNode* a, AStarNode* b) { return *a > *b; };
    std::priority_queue<AStarNode*, std::vector<AStarNode*>, decltype(cmp)> openSet(cmp);
    
    // Track visited nodes
    std::set<std::pair<int, int>> closedSet;
    std::map<std::pair<int, int>, AStarNode*> allNodes;
    
    // Start node
    double h = heuristic(startGX, startGY, goalGX, goalGY);
    AStarNode* startNode = new AStarNode(startGX, startGY, 0.0, h);
    openSet.push(startNode);
    allNodes[{startGX, startGY}] = startNode;
    
    AStarNode* goalNode = nullptr;
    
    while (!openSet.empty()) {
        AStarNode* current = openSet.top();
        openSet.pop();
        
        // Check if goal reached
        if (current->x == goalGX && current->y == goalGY) {
            goalNode = current;
            break;
        }
        
        // Mark as visited
        closedSet.insert({current->x, current->y});
        
        // Explore neighbors
        auto neighbors = getNeighbors(current->x, current->y, true);
        for (const auto& [nx, ny] : neighbors) {
            if (closedSet.count({nx, ny})) {
                continue;  // Already visited
            }
            
            double moveCost = getCost(nx, ny);
            if (moveCost > 500.0) {
                continue;  // Obstacle, skip
            }
            
            // Diagonal cost is sqrt(2), straight is 1.0
            bool isDiagonal = (nx != current->x && ny != current->y);
            double edgeCost = isDiagonal ? 1.414 * moveCost : moveCost;
            
            double newG = current->g + edgeCost;
            double newH = heuristic(nx, ny, goalGX, goalGY);
            
            auto it = allNodes.find({nx, ny});
            if (it == allNodes.end()) {
                // New node
                AStarNode* newNode = new AStarNode(nx, ny, newG, newH, current);
                openSet.push(newNode);
                allNodes[{nx, ny}] = newNode;
            } else if (newG < it->second->g) {
                // Better path found
                it->second->g = newG;
                it->second->f = newG + it->second->h;
                it->second->parent = current;
                openSet.push(it->second);  // Re-add with updated cost
            }
        }
    }
    
    // Reconstruct path
    if (goalNode) {
        AStarNode* node = goalNode;
        while (node) {
            double worldX, worldY;
            map_->gridToWorld(node->x, node->y, worldX, worldY);
            path.push_back(Waypoint(worldX, worldY));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
    }
    
    // Cleanup
    for (auto& pair : allNodes) {
        delete pair.second;
    }
    
    return path;
}

bool AStarPlanner::isPathClear(double x1, double y1, double x2, double y2)
{
    // Bresenham's line algorithm to check if path is clear
    int gx1, gy1, gx2, gy2;
    map_->worldToGrid(x1, y1, gx1, gy1);
    map_->worldToGrid(x2, y2, gx2, gy2);
    
    int dx = std::abs(gx2 - gx1);
    int dy = std::abs(gy2 - gy1);
    int sx = (gx1 < gx2) ? 1 : -1;
    int sy = (gy1 < gy2) ? 1 : -1;
    int err = dx - dy;
    
    int x = gx1;
    int y = gy1;
    
    while (true) {
        double worldX, worldY;
        map_->gridToWorld(x, y, worldX, worldY);
        
        if (map_->isObstacle(worldX, worldY)) {
            return false;
        }
        
        if (x == gx2 && y == gy2) {
            break;
        }
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
    
    return true;
}

// ==================== Spiral Explorer Implementation ====================

SpiralExplorer::SpiralExplorer(GridMap* map) : map_(map) {}

std::vector<std::pair<double, double>> SpiralExplorer::generateSpiralPattern(
    double centerX, double centerY, double maxRadius, int numPoints)
{
    std::vector<std::pair<double, double>> points;
    
    // Archimedean spiral: r = a * theta
    double a = maxRadius / (2.0 * M_PI * 3.0);  // 3 full rotations
    double dTheta = (2.0 * M_PI * 3.0) / numPoints;
    
    for (int i = 0; i < numPoints; i++) {
        double theta = i * dTheta;
        double r = a * theta;
        
        if (r > maxRadius) {
            break;
        }
        
        double x = centerX + r * std::cos(theta);
        double y = centerY + r * std::sin(theta);
        
        points.push_back({x, y});
    }
    
    return points;
}

std::vector<Waypoint> SpiralExplorer::generateSpiralWaypoints(
    double centerX, double centerY, double maxRadius, int numWaypoints)
{
    std::vector<Waypoint> waypoints;
    
    auto spiralPoints = generateSpiralPattern(centerX, centerY, maxRadius, numWaypoints * 2);
    
    // Filter to valid, explorable points
    for (const auto& [x, y] : spiralPoints) {
        int gx, gy;
        map_->worldToGrid(x, y, gx, gy);
        
        if (!map_->isInBounds(gx, gy)) {
            continue;
        }
        
        if (!map_->isObstacle(x, y)) {
            Waypoint wp(x, y, 1);
            waypoints.push_back(wp);
            
            if (waypoints.size() >= static_cast<size_t>(numWaypoints)) {
                break;
            }
        }
    }
    
    explorationWaypoints_ = waypoints;
    return waypoints;
}

double SpiralExplorer::calculateExplorationValue(double x, double y, 
                                                   double currentX, double currentY)
{
    // Higher value = more valuable to explore
    double value = 0.0;
    
    // Factor 1: Distance (prefer closer unexplored areas)
    double dist = std::sqrt((x - currentX) * (x - currentX) + (y - currentY) * (y - currentY));
    double distValue = 1.0 / (1.0 + dist / 100.0);  // Normalize by 100cm
    
    // Factor 2: Exploration status (prefer unexplored)
    double exploredValue = 1.0;
    if (map_->isExplored(x, y)) {
        exploredValue = 0.3;  // Already explored
    }
    
    // Factor 3: Visit count (avoid revisiting)
    int visitCount = map_->getVisitCount(x, y);
    double visitValue = 1.0 / (1.0 + visitCount);
    
    // Factor 4: Check surrounding area for unexplored cells
    int unexploredNeighbors = 0;
    const double searchRadius = 20.0;  // cm
    for (double dx = -searchRadius; dx <= searchRadius; dx += map_->getResolution()) {
        for (double dy = -searchRadius; dy <= searchRadius; dy += map_->getResolution()) {
            if (!map_->isExplored(x + dx, y + dy) && !map_->isObstacle(x + dx, y + dy)) {
                unexploredNeighbors++;
            }
        }
    }
    double frontierValue = std::min(1.0, unexploredNeighbors / 10.0);
    
    // Combined value
    value = distValue * 0.3 + exploredValue * 0.3 + visitValue * 0.2 + frontierValue * 0.2;
    
    return value;
}

Waypoint SpiralExplorer::getNextExplorationTarget(double currentX, double currentY,
                                                    const std::set<int>& visitedAruco)
{
    // Find the best unexplored target
    Waypoint bestWaypoint(currentX, currentY, 0);
    double bestValue = -1.0;
    
    // Evaluate all potential waypoints
    if (explorationWaypoints_.empty()) {
        // Generate new spiral if empty
        generateSpiralWaypoints(currentX, currentY, 150.0, 50);  // 150cm radius, 50 waypoints
    }
    
    for (auto& wp : explorationWaypoints_) {
        if (wp.reached) {
            continue;
        }
        
        double value = calculateExplorationValue(wp.x, wp.y, currentX, currentY);
        
        if (value > bestValue) {
            bestValue = value;
            bestWaypoint = wp;
        }
    }
    
    return bestWaypoint;
}

void SpiralExplorer::updatePriorities(const std::set<int>& visitedAruco,
                                       const std::map<int, std::pair<double, double>>& arucoPositions)
{
    // Adjust waypoint priorities based on ArUco marker locations
    for (auto& wp : explorationWaypoints_) {
        // Check proximity to visited ArUco markers
        bool nearVisited = false;
        for (int id : visitedAruco) {
            auto it = arucoPositions.find(id);
            if (it != arucoPositions.end()) {
                double dx = wp.x - it->second.first;
                double dy = wp.y - it->second.second;
                double dist = std::sqrt(dx * dx + dy * dy);
                
                if (dist < 50.0) {  // Within 50cm of visited marker
                    nearVisited = true;
                    break;
                }
            }
        }
        
        if (nearVisited) {
            wp.priority = 0;  // Lower priority near visited markers
        } else {
            wp.priority = 10;  // Higher priority elsewhere
        }
    }
}

// ==================== Path Smoother Implementation ====================

std::vector<Waypoint> PathSmoother::smoothPath(const std::vector<Waypoint>& path, GridMap* map)
{
    if (path.size() <= 2) {
        return path;
    }
    
    std::vector<Waypoint> smoothed;
    smoothed.push_back(path[0]);
    
    size_t i = 0;
    while (i < path.size() - 1) {
        size_t farthest = i + 1;
        
        // Find farthest visible point
        for (size_t j = i + 2; j < path.size(); j++) {
            AStarPlanner planner(map);
            if (planner.isPathClear(path[i].x, path[i].y, path[j].x, path[j].y)) {
                farthest = j;
            } else {
                break;
            }
        }
        
        smoothed.push_back(path[farthest]);
        i = farthest;
    }
    
    return smoothed;
}

std::vector<Waypoint> PathSmoother::simplifyPath(const std::vector<Waypoint>& path, double epsilon)
{
    if (path.size() <= 2) {
        return path;
    }
    
    // Douglas-Peucker algorithm
    // Find point with maximum distance from line
    double maxDist = 0.0;
    size_t maxIndex = 0;
    
    double x1 = path.front().x;
    double y1 = path.front().y;
    double x2 = path.back().x;
    double y2 = path.back().y;
    
    for (size_t i = 1; i < path.size() - 1; i++) {
        double px = path[i].x;
        double py = path[i].y;
        
        // Point-to-line distance
        double dx = x2 - x1;
        double dy = y2 - y1;
        double norm = std::sqrt(dx * dx + dy * dy);
        
        double dist = 0.0;
        if (norm > 0.0) {
            dist = std::abs(dy * px - dx * py + x2 * y1 - y2 * x1) / norm;
        }
        
        if (dist > maxDist) {
            maxDist = dist;
            maxIndex = i;
        }
    }
    
    // If max distance is greater than epsilon, recursively simplify
    if (maxDist > epsilon) {
        // Split and simplify
        std::vector<Waypoint> left(path.begin(), path.begin() + maxIndex + 1);
        std::vector<Waypoint> right(path.begin() + maxIndex, path.end());
        
        auto leftSimplified = simplifyPath(left, epsilon);
        auto rightSimplified = simplifyPath(right, epsilon);
        
        // Combine (remove duplicate middle point)
        leftSimplified.pop_back();
        leftSimplified.insert(leftSimplified.end(), rightSimplified.begin(), rightSimplified.end());
        
        return leftSimplified;
    } else {
        // Base case: just keep endpoints
        return {path.front(), path.back()};
    }
}

