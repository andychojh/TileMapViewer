#pragma once

#include "WalkNode.h"
#include "widget.h"
#include <vector>
#include <tuple>
#include <QString>
#include "point.h"
#include "place.h"

class widget;  

constexpr double EARTH_RADIUS_M = 6371000.0;
constexpr int64_t SCALE = 10000000;
using Coord = std::pair<double, double>;


// Navigator class
class Navigator {
public:
    // === Data ===
    vector<Point*> points;
    vector<WalkWay> walkWays; 
    vector<WalkNode> walkNodes; 

    // === Utility Functions ===
    double deg2rad(double deg);
    double haversine(double lng1, double lat1, double lng2, double lat2);
    const WalkNode* findClosestNode(const std::vector<WalkNode>& nodes, double startLng, double startLat);
    const Point* findClosestPoint(double startLng, double startLat);
    
    // === Path Computation ===
    const Point* startPoint; 
    const Point* destinationPoint; 
    TurnDirection getTurnDirection(const WalkNode& A, const WalkNode& B, const WalkNode& C);
    std::vector<WalkNode> create_nodes(std::vector<WalkWay>& walkWays);
    std::vector<Point*> load_points(QString& filePath);
    std::vector<WalkWay> load_walkWay(QString& filePath);
    std::vector<int> dijkstra(std::vector<WalkNode>& walkNodes, int start_id, int end_id);
    std::vector<WayTrack> wayTrackerWithDirection(
        const std::vector<int>& path,
        const std::vector<WalkNode>& walkNodes, 
        const std::vector<WalkWay>& walkWays);


    vector<int> findPath(double startLng, double startLat, double detLng, double detLat);
    
    
    // === Main Entry ===
    void run();
};



