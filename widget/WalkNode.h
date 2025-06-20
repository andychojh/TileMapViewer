#pragma once

#include <vector>
#include <QString>

using namespace std; 


struct Connection {
    int target_wayId;
    int linked_indices[2];
};


// Road를 나타내는 구조체 
struct WalkWay {
    int wayId;
    QString name;
    vector<pair<double, double>> coordinates;
    vector<Connection> connections;
};


// Road상 point를 나타내는 구조체 
struct WalkNode {
    int id;
    double lng, lat;
    vector<pair<int, int>> walkPos;         // Walkway상에서 position 
    vector<int> neighbors;                  // 연결된 노드 id들 

    double dist = numeric_limits<double>::infinity();       // 시작점에서 노드까지의 최단거리 
    int prev = -1;          // 이전 노드의 id 
    bool visited = false;

    bool operator==(const WalkNode& rhs) const {
        return lng == rhs.lng && lat == rhs.lat;
    }
};


// Direction enum
enum class TurnDirection { LEFT, RIGHT, STRAIGHT };


// 경로 WayTrack 기록 구조체 
struct WayTrack {
    int wayId; 
    double distance;
    TurnDirection direction;          
    int tpOffset;                   // Turning Point Offset in Path
    QString name = "noname";          // optional
    bool hasName = false;             // name이 있는지 여부    
    bool highlight = false;            // listWidget 클릭시 활성화 
};