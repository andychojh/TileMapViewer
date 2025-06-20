#include <vector>
#include <map>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <cmath>
#include <queue>
#include "WalkNode.h"
#include "Navigator.h"


using namespace std;

// === Utility Functions ===
double Navigator::deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

double Navigator::haversine(double lng1, double lat1, double lng2, double lat2) {
    double dlat = deg2rad(lat2 - lat1);
    double dlng = deg2rad(lng2 - lng1);
    double a = sin(dlat / 2) * sin(dlat / 2) +
        cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
        sin(dlng / 2) * sin(dlng / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS_M * c;
}

const WalkNode* Navigator::findClosestNode(const vector<WalkNode>& nodes, double lng, double lat)
{

    const WalkNode* closestNode = nullptr;
    double minDistance = std::numeric_limits<double>::max();

    for (const WalkNode& node : nodes) {
        double d = haversine(lng, lat, node.lng, node.lat);
        if (d < minDistance) {
            minDistance = d;
            closestNode = &node;
        }
    }

    return closestNode;
}

const Point* Navigator::findClosestPoint(double lng, double lat)
{

    const Point* closestNode = nullptr;
    double minDistance = std::numeric_limits<double>::max();


    for (Point* pt : points) {  // points: std::vector<Point*>
        double d = haversine(lng, lat, pt->coordinates.first, pt->coordinates.second);
        if (d < minDistance) {
            minDistance = d;
            closestNode = pt;
        }
    }

    /*
        for (auto it = points.begin(); it != points.end(); ++it) {
        double d = haversine(lng, lat, (*it)->coordinates.first, (*it)->coordinates.second);
        if (d < minDistance) {
            minDistance = d;
            closestNode = it;
        }
    }*/
    qDebug() << closestNode->name;
    
    return closestNode;
}


// === Path Computation ===
TurnDirection Navigator::getTurnDirection(const WalkNode& A, const WalkNode& B, const WalkNode& C) {
    double angle= 0.0; 

    if (angle < 30.0) return TurnDirection::STRAIGHT;
    
}

// === I/O and Loader ===

vector<Point*> Navigator::load_points(QString& filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qDebug() << "Error: Could not open JSON file: " << file.errorString();
    }

    // JSON 데이터 읽기 
    QByteArray jsonData = file.readAll();
    file.close();

    // JSON 데이터 파싱 
    QJsonParseError jsonError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData, &jsonError);
    if (jsonError.error != QJsonParseError::NoError) {
        qDebug() << "Error: JSON parse error:" << jsonError.errorString();
    }

    // Json 데이터 접근 & 저장 
    vector<Point*> points;
    if (!jsonDoc.isArray()) {
        qDebug() << "Error: JSON document is not an array";
        return points;
    }

    QJsonArray jsonArray = jsonDoc.array();


    for (const QJsonValue& value : jsonArray) {
        if (!value.isObject()) {
            qDebug() << "Warning: skipping non-object JSON element";
            continue;
        }

        QJsonObject obj = value.toObject();

        Point* pt = nullptr;

        // coordinates (필수)
        if (!obj.contains("coordinates")) {
            qDebug() << "Error: no 'coordinates' field";
            continue;
        }

        QJsonArray arr = obj["coordinates"].toArray();
        if (arr.size() < 2 || !arr[0].isDouble() || !arr[1].isDouble()) {
            qDebug() << "Error: 'coordinates' array invalid";
            continue;
        }

        double lng = arr[0].toDouble();
        double lat = arr[1].toDouble();

        // address와 amenity가 있으면 → Place 생성
        bool isPlace = obj.contains("address") && obj.contains("amenity");

        if (isPlace) {
            Place* place = new Place();
            place->coordinates = { lng, lat };

            if (obj.contains("name")) place->name = obj["name"].toString();
            if (obj.contains("address")) place->address = obj["address"].toString();
            if (obj.contains("amenity")) place->amenity = obj["amenity"].toString();

            pt = place;
        }
        else {
            Point* base = new Point();
            if (obj.contains("name")) base->name = obj["name"].toString();;
            base->coordinates = { lng, lat };
            pt = base;
        }

        points.push_back(pt);
    }

    return points;
}


vector<WalkWay> Navigator::load_walkWay(QString& filePath) {

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qDebug() << "Error: Could not open JSON file:" << file.errorString();
    }

    // JSON 데이터 읽기 
    QByteArray jsonData = file.readAll();
    file.close();

    // JSON 데이터 파싱 
    QJsonParseError jsonError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData, &jsonError);
    if (jsonError.error != QJsonParseError::NoError) {
        qDebug() << "Error: JSON parse error:" << jsonError.errorString();
    }

    // JSON 데이터 접근 & 저장 
    vector<WalkWay> walkWays;
    if (jsonDoc.isArray()) {
        QJsonArray jsonArray = jsonDoc.array();

        // 배열의 요소에 접근
        for (int i = 0; i < jsonArray.size(); ++i) {
            QJsonValue arrayElement = jsonArray.at(i);
            if (arrayElement.isObject()) {
                QJsonObject elementObject = arrayElement.toObject();
                QJsonValue wayId = elementObject.value("idx");
                QJsonValue wayName_ = elementObject.value("name");
                QString wayName = (!wayName_.isNull() && !wayName_.isUndefined())
                    ? wayName_.toString()
                    : "noname";
                QJsonValue coordinates = elementObject.value("coordinates");
                QJsonValue connections = elementObject.value("connections");

                QJsonArray coordinatesArray = coordinates.toArray();
                vector<pair<double, double>> coordVector;
                coordVector.reserve(coordinatesArray.size());
                for (const QJsonValue& v : coordinatesArray) {
                    coordVector.push_back(pair(v[0].toDouble(), v[1].toDouble()));
                }


                QJsonArray connectionsArray = connections.toArray();
                vector<Connection> conns;
                for (const QJsonValue& con : connectionsArray) {
                    QJsonArray arr = con.toObject().value("linked_indices").toArray();
                    Connection connection;
                    connection.target_wayId = con.toObject().value("way_id").toInt();
                    for (int i = 0; i < 2; i++) connection.linked_indices[i] = arr[i].toInt();
                    conns.push_back(connection);
                }

                // WalkWay 구조체 생성 
                WalkWay walkWay;
                walkWay.name = wayName;
                walkWay.wayId = wayId.toInt();
                walkWay.coordinates = coordVector;
                walkWay.connections = conns;
                walkWays.push_back(walkWay);
            }
        }
        return walkWays;
    }
    else {
        qDebug() << "Error: Root element is not an array.";
    }
}

vector<WalkNode> Navigator::create_nodes(vector<WalkWay>& walkWays) {
    vector<WalkNode> walkNodes;
  
    return walkNodes;
}

// == 최단경로 알고리즘 == 
vector<int> Navigator::dijkstra(vector<WalkNode>& walkNodes, int start_id, int end_id) {

    for (auto& node : walkNodes) { // Reset fields before each search
        node.dist = numeric_limits<double>::infinity();
        node.prev = -1;
        node.visited = false;
    }

    // Priorirty_queue (가중치, node_id)
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

    walkNodes[start_id].dist = 0.0;
    pq.push({ 0.0, start_id });

    while (!pq.empty()) {
        pair<double, int> top = pq.top();       // 우선 순위 node explore 
        pq.pop();
        double curr_dist = top.first;
        int curr_id = top.second;
        WalkNode& curr = walkNodes[curr_id];

        if (curr.visited) continue;         // 이미 방문한 node면 다음 우선 순위 노드로 진행 
        curr.visited = true;

        if (curr_id == end_id) break;       // 목적지 node에 도착시 종료 

        //qDebug() << "Explore neighbour of node" << curr_id; 
        for (int nei_id : curr.neighbors) {
            //qDebug() << "Neighbour " << nei_id; 
            // 이웃 순회 
            WalkNode& nei = walkNodes[nei_id];
            double weight = haversine(curr.lng, curr.lat, nei.lng, nei.lat);

            if (nei.dist > curr.dist + weight) {
                // 최단거리 업데이트 
                nei.dist = curr.dist + weight;
                nei.prev = curr_id;
                pq.push({ nei.dist, nei_id });
            }
        }
    }

    // 경로 추적 
    vector<int> path;
    int now = end_id;
    if (walkNodes[now].prev == -1 && now != start_id) {
        qDebug() << "No path found!";
        return path;      // 경로 없음 
    }
    while (now != -1) {
        path.push_back(now);
        now = walkNodes[now].prev;
    }
    reverse(path.begin(), path.end());          // 역추적 한 경로 뒤집기       
    return path;
}

 
// 거쳐간 wayId랑, 이동 거리 구하기 
vector<WayTrack> Navigator::wayTrackerWithDirection(const vector<int>& path, const vector<WalkNode>& walkNodes, const vector<WalkWay>& walkWays) {
    vector<WayTrack> wayTrackLog;

    return wayTrackLog;
}


vector<int> Navigator::findPath(double startLng, double startLat, double destLng, double destLat){
    vector<int> path; 
    return path; 
}

void Navigator::run() {
    QString filePath = "Seocho_map_cooked.json";
    QString pointPath = "points.json";
    points = load_points(pointPath);
    walkWays = load_walkWay(filePath);
    walkNodes = create_nodes(walkWays);
}
